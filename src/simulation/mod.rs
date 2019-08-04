mod vehicle;

use core::cmp::Ordering;
use std::collections::{HashMap, HashSet};
use smallvec::SmallVec;
use crate::util::{IdMap, LinearFunc, CubicFunc};
use vehicle::Vehicle;

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
	link_in: usize,
	link_out: usize,
	num_lanes: u8,
	lanes: [(u8, u8); 8]
}

#[derive(Clone)]
struct Lane {
	dist: LinearFunc,
	lat: CubicFunc
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