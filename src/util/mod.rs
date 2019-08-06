mod idmap;
mod piecewise;

use std::cmp::Ordering;
pub use idmap::IdMap;
pub use piecewise::{LinearFunc, CubicFunc, CubicFuncPiece};

pub fn bubble_sort<T, F>(vec: &mut Vec<T>, cmp: F) where F: Fn(&T, &T) -> Ordering {
    let j = vec.len();
    if j < 2 {
        return;
    }
    let j = j - 1;
    loop {
        let mut in_order = true;
        for i in 0..j {
            if cmp(&vec[i], &vec[i + 1]) == Ordering::Greater {
                vec.swap(i, i + 1);
                in_order = false;
            }
        }
        if in_order {
            break;
        }
    }
}