use smallvec::{SmallVec, smallvec};
use super::vehicle::Vehicle;
use crate::util::{IdMap, LinearFunc, CubicFunc, bubble_sort};

#[derive(Clone)]
pub struct Link {
	pub id: usize,
	pub links_in: Vec<LinkConnection>,
	pub links_out: Vec<LinkConnection>,
	pub length: f32,
	pub lanes: SmallVec<[Lane; 6]>,
	pub speed_limit: f32,
	obstacles: Vec<Obstacle>
}

impl Link {
    pub fn new(id: usize, length: f32, speed_limit: f32) -> Self {
        Self {
			id,
			links_in: vec![],
			links_out: vec![],
			length,
			lanes: smallvec![],
			speed_limit,
			obstacles: vec![]
		}
    }

	pub fn add_veh(&mut self, veh: usize) {
		self.obstacles.insert(0, Obstacle {
			veh,
			pos: 0.0,
			vel: 0.0,
			lat: 0.0,
			half_wid: 0.0,
			lane: 0
		});
	}

	pub fn remove_veh(&mut self, veh: usize) {
		let ind = self.obstacles.iter().rposition(|o| o.veh == veh);
        if let Some(i) = ind {
		    self.obstacles.remove(i);
        }
		// todo: remove from stopline?
	}

    pub fn get_offset_to_link(&self, link: usize) -> f32 {
        self.links_out.iter().find(|c| c.link_out == link).unwrap().offset
    }

	pub fn update_obstacles(&mut self, vehs: &IdMap<Vehicle>) {
		for obst in self.obstacles.iter_mut() {
			*obst = vehs.get(obst.veh).get_obstacle();
		}
		bubble_sort(&mut self.obstacles, |a, b| a.pos.partial_cmp(&b.pos).unwrap());
	}

	pub fn car_follow_model(&self, vehs: &mut IdMap<Vehicle>, links: &IdMap<Link>) {
		let num_obst = self.obstacles.len();
		
		for i in 0..num_obst {
			let veh = vehs.get_mut(self.obstacles[i].veh);
			let lane = self.obstacles[i].lane;
			self.car_follow_inner(i + 1, veh, lane, 0, 0.0, 0.0, links);
		}
	}

	fn car_follow_inner(&self, i: usize, veh: &mut Vehicle, lane: u8, r: usize, offset: f32, dist: f32, links: &IdMap<Link>) {
		let num_obst = self.obstacles.len();
		for j in i..num_obst {
			let obst = &self.obstacles[j];
			if obst.lane == lane {
				veh.follow(dist + obst.pos, obst.vel);
				return;
			}
			
			// TODO: other blockers, next lane, etc.

		}
		// Next link
		if r < veh.link_route.len() {
			if r >= veh.lane_route.len() {
				// Stop before reaching the next link
				veh.stop(dist + self.length);
				return;
			}
			// Search the next link
			let next_link = veh.link_route[r];
			let offset = offset + self.get_offset_to_link(next_link);
			let lane = veh.lane_route[r];
			links.get(next_link).car_follow_inner(0, veh, lane, r + 1, offset, dist + self.length, links);
		}
	}
}

#[derive(Clone, Copy)]
pub struct LinkConnection {
	pub link_in: usize,
	pub link_out: usize,
	pub num_lanes: u8,
	pub lanes: [(u8, u8); 8],
    pub offset: f32
}

#[derive(Clone)]
pub struct Lane {
	pub dist: LinearFunc,
	pub lat: CubicFunc
}

#[derive(Clone, Copy)]
pub struct Obstacle {
	// todo: accomodate 'virtual' vehicles
	pub veh: usize,
	pub pos: f32,
	pub vel: f32,
	pub lat: f32,
	pub half_wid: f32,
	pub lane: u8
}