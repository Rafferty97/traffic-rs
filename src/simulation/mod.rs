mod vehicle;
mod link;

use core::cmp::Ordering;
use smallvec::{SmallVec};
use std::collections::{HashMap, HashSet};
use crate::util::{IdMap, LinearFunc, CubicFunc};
use vehicle::Vehicle;
use link::{Link, Lane, LinkConnection, Obstacle};
pub use vehicle::VehicleState;

pub struct Simulation {
	step: usize,
	step_delta: f32,
	links: IdMap<Link>,
	vehs: IdMap<Vehicle>,
	stoplines: IdMap<StopLine>,
	route_table: HashMap<RouteTableKey, RouteTableEntry>,
	lane_route_period: usize
}

impl Simulation {
	pub fn new(step_delta: f32) -> Self {
		Self {
			step: 0,
			step_delta,
			links: IdMap::new(),
			vehs: IdMap::new(),
			stoplines: IdMap::new(),
			route_table: HashMap::new(),
			lane_route_period: 5
		}
	}

	pub fn add_vehicle(&mut self, id: usize) -> usize {
		let id = self.vehs.insert_free(Vehicle::new(id));
		self.vehs.get_mut(id).unwrap().id = id;
		id
	}

	pub fn set_vehicle_pos(&mut self, id: usize, link: usize, lane: u8, pos: f32) {
		let veh = self.vehs.get_mut(id).unwrap();
		if let Some(old_link) = veh.get_link(0) {
			self.links.get_mut(old_link).unwrap().remove_veh(id);
		}
		veh.set_pos(link, lane, pos);
		veh.update_path(&self.links);
		self.links.get_mut(link).unwrap().add_veh(id);
	}

	pub fn set_vehicle_dest(&mut self, id: usize, link: usize) {
		let src_link = self.vehs.get(id).unwrap().get_link(0).unwrap();
		let route = self.find_route(src_link, link);
		self.vehs.get_mut(id).unwrap().set_route(route);
	}

	pub fn add_link(&mut self, id: usize, length: f32, speed_limit: f32) {
		self.links.insert(id, Link::new(id, length, speed_limit));
	}

	pub fn add_lane(&mut self, link: usize, dist_func: LinearFunc, lat_func: CubicFunc) {
		self.links.get_mut(link).unwrap().lanes.push(Lane {
			dist: dist_func,
			lat: lat_func
		});
	}

	pub fn add_connection(&mut self, src_link: usize, dst_link: usize, lanes: &str, offset: f32) {
		let lanes = lanes.split(";").map(|s| {
			let mut p = s.split(":");
			let l1: u8 = p.next().unwrap().parse().unwrap();
			let l2: u8 = p.next().unwrap().parse().unwrap();
			(l1, l2)
		}).collect::<SmallVec<_>>();
		let conn = LinkConnection {
			link_in: src_link,
			link_out: dst_link,
			lanes,
			offset
		};
		self.links.get_mut(src_link).unwrap().links_out.push(conn.clone());
		self.links.get_mut(dst_link).unwrap().links_in.push(conn);
	}

	pub fn add_conflict(&mut self, stop1: usize, stop2: usize, priority: Ordering, max_pos: f32) {
		let stopline = self.stoplines.get_mut(stop1).unwrap();
		stopline.conflicts.push(Conflict {
			stopline: stop2,
			priority,
			max_pos
		});
	}

	pub fn get_step(&self) -> usize {
		self.step
	}

	pub fn step(&mut self) {
		// Update lane decisions
		let per = self.lane_route_period;
		for veh in self.vehs.iter_mut() {
			if veh.id % per != self.step % per { continue; }
			veh.lane_decisions(&self.links);
		}

		// Car-following model
		for link in self.links.iter_mut() {
			link.update_obstacles(&self.vehs);
		}
		for link in self.links.iter() {
			link.car_follow_model(&mut self.vehs, &self.links);
		}
		
		// Stop lines
		// todo: There must be a better way...
		let mut stopline = StopLine::default();
		for id in 0..self.stoplines.len() {
			if !self.stoplines.has_key(id) {
				continue;
			}
			std::mem::swap(&mut stopline, self.stoplines.get_mut(id).unwrap());
			stopline.step(&mut self.vehs, &self.links, &self.stoplines);
			std::mem::swap(&mut stopline, self.stoplines.get_mut(id).unwrap());
		}

		// Integrate vehicles
		for veh in self.vehs.iter_mut() {
			veh.apply_speedlimit(&self.links);
			veh.integrate(self.step_delta, &mut self.links);
		}
		
		// Remove exited vehicles
		self.vehs.remove_where(|v| v.get_link(0).is_none());

		self.step += 1;
	}

	pub fn get_vehicle_states<'a>(&'a self) -> impl Iterator<Item=VehicleState> + 'a {
		self.vehs.iter().map(|v| v.get_state())
	}

	pub fn find_route(&mut self, src_link: usize, dst_link: usize) -> Vec<usize> {
		let mut vec = vec![];
		vec.push(src_link);
		let mut link = src_link;
		while link != dst_link {
			let next = Self::min_dist_to_link(&self.links, &mut self.route_table, link, dst_link);
			if next.dist == std::f32::INFINITY {
				break;
			}
			link = next.next_link;
			vec.push(link);
		}
		vec
	}

	fn min_dist_to_link(links: &IdMap<Link>, route_table: &mut HashMap<RouteTableKey, RouteTableEntry>,
		src_link: usize, dst_link: usize) -> RouteTableEntry
	{
		if src_link == dst_link {
			return RouteTableEntry {
				next_link: !0,
				dist: 0.0
			};
		}

		let key = RouteTableKey { src_link, dst_link };
		if let Some(entry) = route_table.get(&key) {
			return *entry;
		} else {
			let mut next_link = !0;
			let mut dist = std::f32::INFINITY;
			for conn in links.get(src_link).unwrap().links_out.iter() {
				let next_entry = Self::min_dist_to_link(links, route_table, conn.link_out, dst_link);
				if next_entry.dist < dist {
					next_link = conn.link_out;
					dist = next_entry.dist;
				}
			}
			let entry = RouteTableEntry { next_link, dist };
			route_table.insert(key, entry);
			return entry;
		}
	}

	//pub fn add_link()
}

#[derive(Clone, Copy, Hash, PartialEq, Eq)]
struct RouteTableKey {
	pub src_link: usize,
	pub dst_link: usize
}

#[derive(Clone, Copy)]
struct RouteTableEntry {
	pub next_link: usize,
	pub dist: f32
}

#[derive(Clone)]
struct StopLine {
	id: usize,
	link: usize,
	lane: u8,
	pos: f32,
	len: f32,
	kind: StopLineType,
	sight_pos: f32,
	conflicts: Vec<Conflict>,
	committed_vehs: HashSet<usize>,
	time_until_enter: f32,
	min_arrival: usize,
	clear_before: f32
}

impl StopLine {
	fn default() -> Self {
		Self {
			id: 0,
			link: 0,
			lane: 0,
			pos: 0.0,
			len: 0.0,
			kind: StopLineType::None,
			sight_pos: 0.0,
			conflicts: vec![],
			committed_vehs: HashSet::new(),
			time_until_enter: 0.0,
			min_arrival: 0,
			clear_before: 0.0
		}
	}

	fn step(&mut self, vehs: &mut IdMap<Vehicle>, links: &IdMap<Link>, stoplines: &IdMap<StopLine>) {
		// Reset statistics, remove cleared vehicles
		self.time_until_enter = std::f32::INFINITY;
		self.min_arrival = std::usize::MAX;
		self.clear_before = std::f32::INFINITY;
		let mut cleared_vehs = vec![];
		for vid in self.committed_vehs.iter().cloned() {
			if let Some(veh) = vehs.get(vid) {
				if veh.link == self.link {
					let pos = veh.pos - 0.5 * veh.len;
					if pos < self.clear_before {
						self.clear_before = pos;
					}
					continue;
				}
				if !veh.get_links().contains(&self.link) {
					cleared_vehs.push(vid);
					continue;
				}
				self.clear_before = 0.0;
			} else {
				cleared_vehs.push(vid);
			}
		}
		for vid in cleared_vehs.iter() {
			self.committed_vehs.remove(vid);
		}
		// Apply stopline to upstream vehicles
		let link = links.get(self.link).unwrap();
		for vid in link.get_vehicles().rev() {
			let veh = vehs.get_mut(vid).unwrap();
			if veh.lane != self.lane || veh.pos > self.pos {
				continue;
			}
			if self.apply_to_veh(veh, self.pos, stoplines) {
				return;
			}
		}
		for conn in link.links_in.iter() {
			let prev_link = links.get(conn.link_in).unwrap();
			self.apply_to_link(prev_link, &[self.link], self.pos + prev_link.length, vehs, links, stoplines);
		}
	}

	fn apply_to_link(&mut self, link: &Link, route: &[usize], pos: f32, vehs: &mut IdMap<Vehicle>, links: &IdMap<Link>, stoplines: &IdMap<StopLine>) {
		let i = route.len();
		for vid in link.get_vehicles().rev() {
			let veh = vehs.get_mut(vid).unwrap();
			if veh.get_lane(i) != Some(self.lane) {
				continue;
			}
			if veh.get_links().len() < route.len() + 1 {
				continue;
			}
			if &veh.get_links()[1..(route.len() + 1)] != route {
				continue;
			}
			if self.apply_to_veh(veh, pos, stoplines) {
				return;
			}
		}
		for conn in link.links_in.iter() {
			let prev_link = links.get(conn.link_in).unwrap();
			let mut new_route = Vec::with_capacity(route.len() + 1);
			new_route.push(link.id);
			new_route.extend(route);
			self.apply_to_link(prev_link, &new_route, pos + prev_link.length, vehs, links, stoplines);
		}
	}

	fn apply_to_veh(&mut self, veh: &mut Vehicle, pos: f32, stoplines: &IdMap<StopLine>) -> bool {
		// todo:
		// - Copy code for this function from C#
		// - Calculate time_until_enter for stopline
		// - Commits, ensuring they are cleared properly
		// - Traffic light control
		// - Ensuring other fields of stopline are calculated (e.g. clear_before)

		// If committed, keep going
		if self.committed_vehs.contains(&veh.id) {
			return false;
		}

		// If before the stop sign, slow down
		if veh.pos < pos - 6.0 {
			veh.stop(pos);
			return true;
		}

		// If not clear, stop
		if !self.is_clear(stoplines) {
			veh.stop(pos);
			return true;
		}

		// Commit
		//if veh.pos > pos - 6.0 {
			self.committed_vehs.insert(veh.id);
			self.clear_before = 0.0;
		//}
		
		false
	}

	fn is_clear(&self, stoplines: &IdMap<StopLine>) -> bool {
		self.conflicts.iter().all(|c| {
			let stopline = stoplines.get(c.stopline).unwrap();
			stopline.clear_before >= c.max_pos
		})
	}
}

pub struct StopLineBuilder {
	id: usize,
	link: usize,
	lane: u8,
	pos: f32,
	len: Option<f32>,
	kind: Option<StopLineType>,
	sight_dist: Option<f32>,
	conflicts: Vec<Conflict>
}

impl StopLineBuilder {
	pub fn new(id: usize, link: usize, lane: u8, pos: f32) -> Self {
		Self {
			id,
			link,
			lane,
			pos,
			len: None,
			kind: None,
			sight_dist: None,
			conflicts: vec![]
		}
	}

	pub fn with_length(mut self, len: f32) -> Self {
		self.len = Some(len);
		self
	}

	pub fn of_type(mut self, kind: StopLineType) -> Self {
		self.kind = Some(kind);
		self
	}

	pub fn with_sight_dist(mut self, dist: f32) -> Self {
		self.sight_dist = Some(dist);
		self
	}

	pub fn conflicts_with(mut self, stopline: usize, priority: Ordering, max_pos: f32) -> Self {
		self.conflicts.push(Conflict {
			stopline,
			priority,
			max_pos
		});
		self
	}

	pub fn add_to_simulation(self, simulation: &mut Simulation) {
		let link = simulation.links.get(self.link).unwrap();
		let len = self.len.unwrap_or(link.length - self.pos);
		let kind = self.kind.expect("Stopline type not specified.");
		let sight_pos = self.pos - self.sight_dist.unwrap_or(50.0);
		simulation.stoplines.insert(self.id, StopLine {
			id: self.id,
			link: self.link,
			lane: self.lane,
			pos: self.pos,
			len,
			kind,
			sight_pos,
			conflicts: self.conflicts,
			committed_vehs: HashSet::new(),
			time_until_enter: 0.0,
			min_arrival: std::usize::MAX,
			clear_before: 0.0
		});
	}
}

#[derive(Clone, Copy)]
pub enum StopLineType {
	None,
	Giveway,
	Stop,
	TrafficLight {
		state: TrafficLightState
	}
}

impl std::str::FromStr for StopLineType {
	type Err = ();

	fn from_str(s: &str) -> Result<StopLineType, ()> {
		match s {
			"none" => Ok(StopLineType::None),
			"giveway" => Ok(StopLineType::Giveway),
			"stop" => Ok(StopLineType::Stop),
			"light" => Ok(StopLineType::TrafficLight {
				state: TrafficLightState::Red
			}),
			_ => Err(())
		}
	}
}

#[derive(Clone, Copy)]
enum TrafficLightState {
	Green,
	Amber,
	Red
}

#[derive(Clone, Copy)]
struct Conflict {
	stopline: usize,
	priority: Ordering,
	max_pos: f32
}