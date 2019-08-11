mod idmap;
mod piecewise;

use std::cmp::Ordering;
pub use idmap::IdMap;
pub use piecewise::{LinearFunc, CubicFunc, CubicFuncPiece};

pub fn insertion_sort<T, F>(vec: &mut Vec<T>, cmp: F) where F: Fn(&T, &T) -> Ordering {
    let len = vec.len();
    if len <= 1 {
        return;
    }
    for i in 1..len {
        for j in (0..i).rev() {
            match cmp(&vec[j], &vec[j + 1]) {
                Ordering::Greater => vec.swap(j, j + 1),
                _ => break
            }
        }
    }
}