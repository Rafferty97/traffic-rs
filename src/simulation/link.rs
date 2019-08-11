use smallvec::{SmallVec, smallvec};
use super::vehicle::Vehicle;
use crate::util::{IdMap, LinearFunc, CubicFunc, insertion_sort};

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

	pub fn get_lane_connections<'a>(&'a self, link: usize) -> impl Iterator<Item=(u8, u8)> + 'a {
		self.links_out.iter()
			.find(|c| c.link_out == link)
			.map(|c| &c.lanes)
			.as_ref()
			.unwrap()
			.iter()
			.cloned()
	}

	pub fn update_obstacles(&mut self, vehs: &IdMap<Vehicle>) {
		for obst in self.obstacles.iter_mut() {
			*obst = vehs.get(obst.veh).get_obstacle();
		}
		insertion_sort(&mut self.obstacles, |a, b| a.pos.partial_cmp(&b.pos).unwrap());
	}

	pub fn get_vehicles<'a>(&'a self) -> impl DoubleEndedIterator<Item=usize> + 'a {
		// todo: change to filter_map when other obstacles are supported
		self.obstacles.iter().map(|o| o.veh)
	}

	pub fn get_lat(&self, lane: u8, pos: f32) -> f32 {
		self.lanes[lane as usize].lat.get_y(pos)
	}

	pub fn car_follow_model(&self, vehs: &mut IdMap<Vehicle>, links: &IdMap<Link>) {
		let num_obst = self.obstacles.len();
		
		for i in 0..num_obst {
			let veh = vehs.get_mut(self.obstacles[i].veh);
			self.car_follow_inner(i + 1, veh, veh.lane, 0, 0.0, 0.0, links);
		}
	}
	
	fn car_follow_inner(&self, i: usize, veh: &mut Vehicle, lane: u8, r: usize, offset: f32, dist: f32, links: &IdMap<Link>) {
		let num_obst = self.obstacles.len();
		for j in i..num_obst {
			let obst = &self.obstacles[j];
			// Follow if in same lane
			if obst.lane == lane {
				veh.follow(dist + obst.pos, obst.vel);
				return;
			}
			// Follow if blocking path
			let pos = obst.pos - (0.5 * veh.len + 1.0);
			let on_curr_path = veh.path.is_some()
				&& pos + dist <= veh.path.unwrap().max_x;
			let (lat, halfwid) = if on_curr_path {
				let path = veh.path.unwrap();
				let mut lat = path.get_y(pos + dist);
				let mut halfwid = 0.5 * veh.wid;
				if veh.changing_lanes {
					let half_delta = 0.5 * (path.get_y2() - lat);
					lat += half_delta;
					halfwid += half_delta.abs();
				}
				lat += offset;
				(lat, halfwid)
			} else {
				(self.get_lat(lane, pos), 0.5 * veh.wid)
			};
			let gap = (lat - obst.lat).abs() - (halfwid + obst.half_wid);
			if gap < 0.5 {
				veh.follow(dist + obst.pos, obst.vel);
			}
		}
		// Next link
		let r = r + 1;
		if let Some(next_link) = veh.get_link(r) {
			if let Some(next_lane) = veh.get_lane(r) {
				if next_lane != !0 {
					// Search the next link
					let offset = offset + self.get_offset_to_link(next_link);
					links.get(next_link).car_follow_inner(0, veh, next_lane, r, offset, dist + self.length, links);
					return;
				}
			}
			// Stop before reaching the next link
			veh.stop(dist + self.length);
		}
	}
}

#[derive(Clone)]
pub struct LinkConnection {
	pub link_in: usize,
	pub link_out: usize,
	pub lanes: SmallVec<[(u8, u8); 8]>,
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