use smallvec::SmallVec;
use crate::util::{CubicFuncPiece};

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
	path: CubicFuncPiece,
	link_route: Vec<usize>,
	lane_route: Vec<u8>,
	lane_dists: Vec<SmallVec<[f32; 8]>>,
	arrival_step: Option<usize>
}