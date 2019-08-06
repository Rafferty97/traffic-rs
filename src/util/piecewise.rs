use std::cmp::Ordering;

/**
 * Represents a piece-wise linear function that maps an f32 to an f32.
 * */
#[derive(Clone)]
pub struct LinearFunc {
    pieces: Vec<LinearFuncPiece>
}

#[derive(Clone, Copy)]
struct LinearFuncPiece {
    min_x: f32,
    max_x: f32,
    m: f32,
    b: f32
}

impl LinearFunc {
    pub fn from_points(points: &[(f32, f32)]) -> Self {
        let mut points = points.iter();
        let mut pieces = vec![];

        let (mut x1, mut y1) = points.next().unwrap();
        while let Some((x2, y2)) = points.next() {
            pieces.push(LinearFuncPiece {
                min_x: x1,
                max_x: *x2,
                m: (y2 - y1) / (x2 - x1),
                b: y1
            });
            x1 = *x2;
            y1 = *y2;
        }

        Self {
            pieces
        }
    }

    pub fn from_const(min_x: f32, max_x: f32, y: f32) -> Self {
        Self {
            pieces: vec![LinearFuncPiece {
                min_x,
                max_x,
                m: 0.0,
                b: y
            }]
        }
    }

    pub fn get_y(&self, x: f32) -> f32 {
        let ind = self.pieces.binary_search_by(|p| {
            if x < p.min_x { Ordering::Greater }
            else if x >= p.max_x { Ordering::Less }
            else { Ordering::Equal }
        }).unwrap();
        let piece = self.pieces[ind];
        piece.m * (x - piece.min_x) + piece.b
    }
}

/**
 * Represents a piece-wise linear function that maps an f32 to an f32.
 * */
#[derive(Clone)]
pub struct CubicFunc {
    pieces: Vec<CubicFuncPiece>
}

#[derive(Clone, Copy)]
pub struct CubicFuncPiece {
    pub min_x: f32,
    pub max_x: f32,
    pub y1: f32,
    pub yd: f32
}

impl CubicFunc {
    pub fn from_points(points: &[(f32, f32)]) -> Self {
        let mut points = points.iter();
        let mut pieces = vec![];

        let (mut x1, mut y1) = points.next().unwrap();
        while let Some((x2, y2)) = points.next() {
            pieces.push(CubicFuncPiece {
                min_x: x1,
                max_x: *x2,
                y1,
                yd: y2 - y1
            });
            x1 = *x2;
            y1 = *y2;
        }

        Self {
            pieces
        }
    }

    pub fn get_y_and_dy(&self, x: f32) -> (f32, f32) {
        self.get_piece(x).get_y_and_dy(x)
    }
    
    pub fn get_y(&self, x: f32) -> f32 {
        self.get_piece(x).get_y_and_dy(x).0
    }
    
    pub fn get_dy(&self, x: f32) -> f32 {
        self.get_piece(x).get_y_and_dy(x).1
    }

    pub fn get_piece(&self, x: f32) -> CubicFuncPiece {
        let ind = self.pieces.binary_search_by(|p| {
            if x < p.min_x { Ordering::Greater }
            else if x >= p.max_x { Ordering::Less }
            else { Ordering::Equal }
        });
        let ind = match ind {
            Ok(i) => i,
            Err(i) => if i == 0 { 0 } else { self.pieces.len() - 1 }
        };
        self.pieces[ind]
    }
}

impl CubicFuncPiece {
    pub fn get_y_and_dy(&self, x: f32) -> (f32, f32) {
        if self.yd == 0.0 || x <= self.min_x {
            return (self.y1, 0.0);
        }
        if x >= self.max_x {
            return (self.y1 + self.yd, 0.0);
        }
        let xd = self.max_x - self.min_x;
        let yd = self.yd;
        let t = (x - self.min_x) / xd;
        let u = t * t * (3.0 - 2.0 * t);
        let y = self.y1 + u * yd;
        let dy = (yd / xd) * 6.0 * t * (1.0 - t);
        (y, dy)
    }

    pub fn get_y2(&self) -> f32 {
        self.y1 + self.yd
    }
    
    pub fn get_y(&self, x: f32) -> f32 {
        self.get_y_and_dy(x).0
    }
    
    pub fn get_dy(&self, x: f32) -> f32 {
        self.get_y_and_dy(x).1
    }

    pub fn translate(&self, dx: f32, dy: f32) -> Self {
        Self {
            min_x: self.min_x + dx,
            max_x: self.max_x + dx,
            y1: self.y1 + dy,
            yd: self.yd
        }
    }
}