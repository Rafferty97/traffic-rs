use smallvec::SmallVec;
use super::{Link};
use crate::util::{CubicFuncPiece, IdMap};

const comf_decel: f32 = -2.5;
const max_decel: f32 = -6.0;

#[derive(Clone)]
pub struct Vehicle {
	// Attributes
	id: usize,
	len: f32,
	wid: f32,
	max_acc: f32,
	// State
	link: usize,
	lane: u8,
	changing_lanes: bool,
	pos: f32,
	vel: f32,
	acc: f32,
	path: Option<CubicFuncPiece>,
	link_route: Vec<usize>,
	lane_route: Vec<u8>,
	lane_dists: Vec<SmallVec<[f32; 8]>>,
	arrival_step: Option<usize>,
	// Derived state
	lat: f32,
	dlat: f32
}

pub struct VehicleState {
	pub id: usize,
	pub link: usize,
	pub pos: f32,
	pub vel: f32,
	pub lat: f32,
	pub dlat: f32
}

impl Vehicle {
	pub fn new(id: usize) -> Self {
		Self {
			id,
			len: 4.6,
			wid: 2.0,
			max_acc: 3.0,
			link: !0,
			lane: 0,
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

	pub fn integrate(&mut self, delta: f32, links: &IdMap<Link>) {
		// Integrate position, reset acceleration
		self.vel += self.acc * delta;
		if self.vel < 0.0 {
			self.vel = 0.0;
		}
		self.pos += self.vel * delta; // todo: road curvature
		self.acc = self.max_acc;

		// Update path, lat and vlat
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
			id: self.id,
			link: self.link,
			pos: self.pos,
			vel: self.vel,
			lat: self.lat,
			dlat: self.dlat
		}
	}
}