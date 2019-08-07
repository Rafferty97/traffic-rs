use std::f32;
use smallvec::SmallVec;
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
	pub link_route: Vec<usize>,
	pub lane_route: Vec<u8>,
	pub lane_dists: Vec<SmallVec<[f32; 8]>>,
	pub arrival_step: Option<usize>,
	// Derived state
	pub lat: f32,
	pub dlat: f32
}

pub struct VehicleState {
	pub user_id: usize,
	pub link: usize,
	pub pos: f32,
	pub vel: f32,
	pub lat: f32,
	pub dlat: f32
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
	}

	pub fn get_link(&self) -> Option<usize> {
		if self.link == !0 { None } else { Some(self.link) }
	}

	pub fn set_route(&mut self, route: Vec<usize>) {
		self.link_route = route;
		if self.link_route[0] == self.link {
			self.link_route.remove(0);
		}
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

	pub fn lane_decisions(&mut self, links: &IdMap<Link>) {
		if self.changing_lanes {
			return;
		}

		// Decide lane route
		if self.id % 3 == 0 && links.get(self.link).lanes.len() >= 2 && self.link_route.len() > 0 {
			self.old_lane = self.lane;
			self.lane = 1 - self.lane;
			self.changing_lanes = true;
		}
		while self.lane_route.len() < self.link_route.len() {
			self.lane_route.push(0);
		}

		// Handle lane change path
		if self.changing_lanes {
			let dist = 40.0;
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
			let next_link = self.link_route[ind];
			offset += link.get_offset_to_link(next_link);
			link = links.get(next_link);
			ind += 1;
		}
		let lane = if ind == 0 { self.lane } else { self.lane_route[ind - 1] };
		link.get_lat(lane, pos) - offset
	}

	pub fn apply_speedlimit(&mut self, links: &IdMap<Link>) {
		let link = links.get(self.link);

		// Current link speed limit
		apply_limit(self, link.speed_limit, 0.0);

		// Next link speed limit
		if let Some(link_id) = self.link_route.get(0) {
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
			if self.link_route.len() > 0 {
				let next_link = self.link_route.remove(0);
				let lat_off = links.get(self.link).get_offset_to_link(next_link);
				self.link = next_link;
				links.get_mut(self.link).add_veh(self.id);
				self.pos -= len;
				self.lane = self.lane_route.remove(0);
				//LaneDists.RemoveAt(0);
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