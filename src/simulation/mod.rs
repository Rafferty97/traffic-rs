mod vehicle;

use core::cmp::Ordering;
use std::collections::{HashMap, HashSet};
use smallvec::{SmallVec, smallvec};
use crate::util::{IdMap, LinearFunc, CubicFunc};
use vehicle::Vehicle;
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
		let veh = Vehicle::new(id);
		self.vehs.insert_free(veh)
	}

	pub fn set_vehicle_pos(&mut self, id: usize, link: usize, lane: u8, pos: f32) {
		self.vehs.get_mut(id).set_pos(link, lane, pos);
	}

	pub fn add_link(&mut self, id: usize, length: f32, speed_limit: f32) {
		self.links.insert(id, Link {
			id,
			links_in: vec![],
			links_out: vec![],
			length,
			lanes: smallvec![],
			speed_limit,
			obstacles: vec![]
		});
	}

	pub fn add_lane(&mut self, link: usize, dist_func: LinearFunc, lat_func: CubicFunc) {
		self.links.get_mut(link).lanes.push(Lane {
			dist: dist_func,
			lat: lat_func
		});
	}

	pub fn step(&mut self) {
		// todo

		for veh in self.vehs.iter_mut() {
			veh.integrate(self.step_delta, &self.links);
		}
	}

	pub fn get_vehicle_states<'a>(&'a self) -> impl Iterator<Item=VehicleState> + 'a {
		self.vehs.iter().map(|v| v.get_state())
	}

	//pub fn add_link()
}

#[derive(Clone)]
struct Link {
	id: usize,
	links_in: Vec<LinkConnection>,
	links_out: Vec<LinkConnection>,
	length: f32,
	lanes: SmallVec<[Lane; 6]>,
	speed_limit: f32,
	obstacles: Vec<Obstacle>
}

#[derive(Clone, Copy)]
struct LinkConnection {
	pub link_in: usize,
	pub link_out: usize,
	pub num_lanes: u8,
	pub lanes: [(u8, u8); 8]
}

#[derive(Clone)]
struct Lane {
	pub dist: LinearFunc,
	pub lat: CubicFunc
}

#[derive(Clone, Copy)]
struct Obstacle {
	pos: f32,
	vel: f32,
	lat: f32,
	half_wid: f32,
	lane: u8
}

#[derive(Clone, Copy, Hash, PartialEq, Eq)]
struct RouteTableKey {
	from_link: usize,
	to_link: usize
}

#[derive(Clone, Copy)]
struct RouteTableEntry {
	next_link: usize,
	dist: f32
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