use std::f32;
use std::cmp::Ordering;
use smallvec::{SmallVec, smallvec};
use super::{Link, Obstacle};
use crate::util::{CubicFuncPiece, IdMap};

const COMF_DECEL: f32 = -2.5;
const MAX_DECEL: f32 = -6.0;

#[derive(Clone)]
pub struct Vehicle {
	// Attributes
	pub id: usize,
	pub user_id: usize,
	pub len: f32,
	pub wid: f32,
	pub max_acc: f32,
	pub follow_c: f32,
	// State
	pub link: usize,
	pub lane: u8,
	pub old_lane: u8,
	pub changing_lanes: bool,
	pub pos: f32,
	pub vel: f32,
	pub acc: f32,
	pub path: Option<CubicFuncPiece>,
	link_route: Vec<usize>,
	lane_route: Vec<u8>,
	lane_dists: Vec<LaneDistances>,
	pub arrival_step: Option<usize>,
	// Derived state
	pub lat: f32,
	pub dlat: f32
}

#[derive(Copy, Clone)]
pub struct VehicleState {
	pub user_id: usize,
	pub link: usize,
	pub pos: f32,
	pub vel: f32,
	pub lat: f32,
	pub dlat: f32
}

#[derive(Clone)]
struct LaneDistances {
	pub lanes: SmallVec<[[f32; 4]; 8]>
}

impl Vehicle {
	pub fn new(user_id: usize) -> Self {
		let max_acc = 3.0;
		Self {
			id: 0,
			user_id,
			len: 4.6,
			wid: 2.0,
			max_acc,
			follow_c: 2.0 * (max_acc * -COMF_DECEL).sqrt(),
			link: !0,
			lane: 0,
			old_lane: 0,
			changing_lanes: false,
			pos: 0.0,
			vel: 0.0,
			acc: 0.0,
			path: None,
			link_route: vec![],
			lane_route: vec![],
			lane_dists: vec![],
			arrival_step: None,
			lat: 0.0,
			dlat: 0.0
		}
	}

	pub fn set_pos(&mut self, link: usize, lane: u8, pos: f32) {
		self.link = link;
		self.pos = pos;
		self.lane = lane;
		self.link_route = vec![self.link];
		self.lane_route = vec![self.lane];
	}

	pub fn get_link(&self, i: usize) -> Option<usize> {
		self.link_route.get(i).cloned()
	}

	pub fn get_links(&self) -> &[usize] {
		&self.link_route
	}

	pub fn get_lane(&self, i: usize) -> Option<u8> {
		self.lane_route.get(i).cloned()
	}

	pub fn set_route(&mut self, route: Vec<usize>) {
		self.link_route = route;
		if self.link_route.get(0) != Some(&self.link) {
			self.link_route.insert(0, self.link);
		}
		self.lane_dists = vec![];
	}

	pub fn get_obstacle(&self) -> Obstacle {
		let pos = self.pos - 0.5 * self.len; // todo: curve
		if self.changing_lanes {
			let half_delta = 0.5 * (self.path.unwrap().get_y2() - self.lat);
			Obstacle {
				veh: self.id,
				pos,
				vel: self.vel,
				lat: self.lat + half_delta,
				half_wid: (0.5 * self.wid) + half_delta.abs(),
				lane: self.lane
			}
		} else {
			Obstacle {
				veh: self.id,
				pos,
				vel: self.vel,
				lat: self.lat,
				half_wid: 0.5 * self.wid,
				lane: self.lane
			}
		}
	}

	fn compute_lane_dists(&mut self, links: &IdMap<Link>) {
		let mut link_iter = self.link_route.iter().rev().cloned();
		let link_id = link_iter.next().unwrap();
		let link = links.get(link_id);
		self.lane_dists = vec![LaneDistances {
			lanes: smallvec![[f32::INFINITY; 4]; link.lanes.len()]
		}];
		let mut succ_link_id = link_id;
		while let Some(link_id) = link_iter.next() {
			let link = links.get(link_id);
			let mut lanes: SmallVec<[[f32; 4]; 8]> = smallvec![[0.0; 4]; link.lanes.len()];
			let succ_lanes = &self.lane_dists.last().unwrap().lanes;
			let mut min_offset = 4;
			for (lane_in, lane_out) in link.get_lane_connections(succ_link_id) {
				for (i, lane) in lanes.iter_mut().enumerate() {
					let offset = (i as isize - lane_in as isize).abs() as usize;
					if offset < min_offset { min_offset = offset; }
					for (a, b) in lane.iter_mut().skip(offset).zip(succ_lanes[lane_out as usize].iter()) {
						if *b > *a { *a = *b; }
					}
				}
			}
			for lane in lanes.iter_mut() {
				for i in 0..(4 - min_offset) {
					lane[i] = lane[i + min_offset] + link.length;
				}
				for i in (4 - min_offset)..4 {
					lane[i] = f32::INFINITY;
				}
			}
			self.lane_dists.push(LaneDistances { lanes });
			succ_link_id = link_id;
		}
		self.lane_dists.reverse();

		/* for d in self.lane_dists.iter() {
			println!("{:?}", d.lanes);
		}
		println!("-----"); */
	}

	fn compare_lanes(&self, i: usize, lane1: u8, lane2: u8) -> Ordering {
		let dists = &self.lane_dists[i].lanes;
		let lane1 = dists.get(lane1 as usize);
		let lane2 = dists.get(lane2 as usize);
		match (lane1, lane2) {
			(None, None) => Ordering::Equal,
			(None, Some(_)) => Ordering::Less,
			(Some(_), None) => Ordering::Greater,
			(Some(lane1), Some(lane2)) => {
				// The best lane is the one requiring the fewest lane-changes
				for i in (0..4).rev() {
					let c = lane1[i].partial_cmp(&lane2[i]).unwrap();
					if c != Ordering::Equal {
						return c;
					}
				}
				Ordering::Equal
			}
		}
	}
	
	pub fn lane_decisions(&mut self, links: &IdMap<Link>) {
		if self.changing_lanes || self.link_route.len() < 2 {
			return;
		}

		if self.lane_dists.len() < self.link_route.len() {
			self.compute_lane_dists(links);
		}

		// Decide lane route
		self.old_lane = self.lane;
		let left_better = if self.lane > 0 {
			self.compare_lanes(0, self.lane - 1, self.lane) == Ordering::Greater
		} else { false };
		let right_better = self.compare_lanes(0, self.lane + 1, self.lane) == Ordering::Greater;
		match (left_better, right_better) {
			(false, false) => {},
			(true, false) => {
				self.lane -= 1;
				self.changing_lanes = true;
				self.lane_route = vec![self.lane];
			},
			(false, true) => {
				self.lane += 1;
				self.changing_lanes = true;
				self.lane_route = vec![self.lane];
			},
			(true, true) => {
				if self.compare_lanes(0, self.lane - 1, self.lane + 1) == Ordering::Less {
					self.lane += 1;
				} else {
					self.lane -= 1;
				}
				self.changing_lanes = true;
				self.lane_route = vec![self.lane];
			}
		}
		while self.lane_route.len() < self.link_route.len() {
			let i = self.lane_route.len() - 1;
			let prev_lane = self.lane_route[i];
			let prev_link = self.link_route[i];
			let link = self.link_route[i + 1];
			let next_lane = links.get(prev_link)
				.get_lane_connections(link)
				.filter_map(|c| if c.0 == prev_lane { Some(c.1) } else { None })
				.fold(!0, |a, b| {
					let c = self.compare_lanes(i, a, b);
					if c == Ordering::Less { b } else { a }
				});
			if next_lane == !0 {
				self.lane_route.resize(self.link_route.len(), !0);
				break;
			}
			self.lane_route.push(next_lane);
		}

		// Handle lane change path
		if self.changing_lanes {
			let dist = 40.0; // todo
			let end_lat = self.get_lat_at_pos(self.pos + dist, links);
			self.path = Some(CubicFuncPiece {
				min_x: self.pos,
				max_x: self.pos + dist,
				y1: self.lat,
				yd: end_lat - self.lat
			});
		}

		// todo: handle case where vlat != 0
	}

	fn get_lat_at_pos(&self, pos: f32, links: &IdMap<Link>) -> f32 {
		let mut ind = 0;
		let mut offset = 0.0;
		let mut pos = pos;
		let mut link = links.get(self.link);
		while pos > link.length {
			pos -= link.length;
			ind += 1;
			let next_link = self.link_route[ind];
			offset += link.get_offset_to_link(next_link);
			link = links.get(next_link);
		}
		link.get_lat(self.lane_route[ind], pos) - offset
	}

	pub fn apply_speedlimit(&mut self, links: &IdMap<Link>) {
		let link = links.get(self.link);

		// Current link speed limit
		apply_limit(self, link.speed_limit, 0.0);

		// Next link speed limit
		if let Some(link_id) = self.link_route.get(1) {
			let next_link = links.get(*link_id);
			apply_limit(self, next_link.speed_limit, link.length - self.pos);
		}

        fn apply_limit(veh: &mut Vehicle, limit: f32, dist: f32) {
            let limit = if dist > 0.0 {
                ((limit * limit) - 2.0 * COMF_DECEL * dist).sqrt()
            } else {
				limit
			};
            let acc = veh.max_acc * (1.0 - (veh.vel / limit).powf(4.0));
            veh.apply_acc(acc);
        }
	}

	pub fn apply_acc(&mut self, acc: f32) {
		if acc < self.acc {
			self.acc = acc;
		}
	}

	pub fn follow(&mut self, pos: f32, vel: f32) {
		// todo: var dist = AdjustPos(pos) - AdjustPos(Pos) - HalfLen;
		let dist = pos - (self.pos + (0.5 * self.len));

		if dist <= 0.0 {
			self.acc = std::f32::MIN;
			return;
		}

		let approach_rate = self.vel - vel;
		let headway = 2.0;
		let mingap = 2.0;
		let ss = mingap + (headway * self.vel) + ((approach_rate * self.vel) / self.follow_c);
		let acc = self.max_acc * (1.0 - (ss / dist).powf(2.0));
		self.apply_acc(acc);
	}

	pub fn stop(&mut self, pos: f32) {
		self.follow(pos, 0.0);
	}

	pub fn integrate(&mut self, delta: f32, links: &mut IdMap<Link>) {
		// Integrate position, reset acceleration
		self.vel += self.acc * delta;
		if self.vel < 0.0 {
			self.vel = 0.0;
		}
		self.pos += self.vel * delta; // todo: road curvature
		self.acc = self.max_acc;
		
		// Advance the link
		let len = links.get(self.link).length;
		if self.pos > len {
			links.get_mut(self.link).remove_veh(self.id);
			self.link_route.remove(0);
			self.lane_route.remove(0);
			self.lane_dists.remove(0);
			if !self.link_route.is_empty() {
				let next_link = self.link_route[0];
				let lat_off = links.get(self.link).get_offset_to_link(next_link);
				self.link = next_link;
				links.get_mut(self.link).add_veh(self.id);
				self.pos -= len;
				self.lane = self.lane_route[0];
				if self.changing_lanes {
					self.path = self.path.map(|p| p.translate(-len, lat_off));
				} else {
					self.path = None;
				}
			} else {
				// Vehicle has reached destination
				self.link = !0;
				return;
			}
		}

		// Update path, lat and vlat
		self.update_path(links);
	}

	pub fn update_path(&mut self, links: &IdMap<Link>) {
		if self.path.map(|p| self.pos > p.max_x).unwrap_or(true) {
			self.path = Some(links.get(self.link)
				.lanes[self.lane as usize]
				.lat.get_piece(self.pos));
			self.changing_lanes = false;
		}
		let (lat, dlat) = self.path.unwrap().get_y_and_dy(self.pos);
		self.lat = lat;
		self.dlat = dlat;
	}

	pub fn get_state(&self) -> VehicleState {
		VehicleState {
			user_id: self.user_id,
			link: self.link,
			pos: self.pos,
			vel: self.vel,
			lat: self.lat,
			dlat: self.dlat
		}
	}
}