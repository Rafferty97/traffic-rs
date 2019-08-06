mod vehicle;
mod link;

use core::cmp::Ordering;
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
	route_table: HashMap<RouteTableKey, RouteTableEntry>
}

impl Simulation {
	pub fn new(step_delta: f32) -> Self {
		Self {
			step: 0,
			step_delta,
			links: IdMap::new(),
			vehs: IdMap::new(),
			stoplines: IdMap::new(),
			route_table: HashMap::new()
		}
	}

	pub fn add_vehicle(&mut self, id: usize) -> usize {
		let id = self.vehs.insert_free(Vehicle::new(id));
		self.vehs.get_mut(id).id = id;
		id
	}

	pub fn set_vehicle_pos(&mut self, id: usize, link: usize, lane: u8, pos: f32) {
		let veh = self.vehs.get_mut(id);
		if let Some(old_link) = veh.get_link() {
			self.links.get_mut(old_link).remove_veh(id);
		}
		veh.set_pos(link, lane, pos);
		self.links.get_mut(link).add_veh(id);
	}

	pub fn set_vehicle_dest(&mut self, id: usize, link: usize) {
		let src_link = self.vehs.get(id).get_link().unwrap();
		let route = self.find_route(src_link, link);
		self.vehs.get_mut(id).set_route(route);
	}

	pub fn add_link(&mut self, id: usize, length: f32, speed_limit: f32) {
		self.links.insert(id, Link::new(id, length, speed_limit));
	}

	pub fn add_lane(&mut self, link: usize, dist_func: LinearFunc, lat_func: CubicFunc) {
		self.links.get_mut(link).lanes.push(Lane {
			dist: dist_func,
			lat: lat_func
		});
	}

	pub fn add_connection(&mut self, src_link: usize, dst_link: usize, lanes: &str, offset: f32) {
		let iter = lanes.split(";").map(|s| {
			let mut p = s.split(":");
			let l1: u8 = p.next().unwrap().parse().unwrap();
			let l2: u8 = p.next().unwrap().parse().unwrap();
			(l1, l2)
		});
		let mut lanes = [(0, 0); 8];
		let mut num_lanes = 0;
		for p in iter {
			lanes[num_lanes] = p;
			num_lanes += 1;
		}
		let conn = LinkConnection {
			link_in: src_link,
			link_out: dst_link,
			num_lanes: num_lanes as u8,
			lanes,
			offset
		};
		self.links.get_mut(src_link).links_out.push(conn);
		self.links.get_mut(dst_link).links_in.push(conn);
	}

	pub fn step(&mut self) {
		// Update lane decisions
		for veh in self.vehs.iter_mut() {
			veh.lane_decisions(&self.links);
		}

		// Car-following model
		for link in self.links.iter_mut() {
			link.update_obstacles(&self.vehs);
		}
		for link in self.links.iter() {
			link.car_follow_model(&mut self.vehs, &self.links);
		}

		// Integrate vehicles
		for veh in self.vehs.iter_mut() {
			// todo: veh.apply_speedlimit(&self.links);
			veh.integrate(self.step_delta, &mut self.links);
		}
		
		// Remove exited vehicles
		self.vehs.remove_where(|v| v.get_link().is_none());

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
			for conn in links.get(src_link).links_out.iter() {
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

#[derive(Clone, Copy)]
enum StopLineType {
	None,
	Giveway,
	Stop,
	TrafficLight {
		state: TrafficLightState
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