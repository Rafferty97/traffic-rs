use std::clone::Clone;

pub struct IdMap<T> where T: Clone {
    vec: Vec<Option<T>>,
    free_slots: Vec<usize>
}

impl<T> IdMap<T> where T: Clone {
    pub fn new() -> Self {
        Self {
            vec: vec![],
            free_slots: vec![]
        }
    }

    pub fn insert(&mut self, id: usize, value: T) {
        if id < self.vec.len() {
            let ind = self.free_slots.iter().position(|x| *x == id).unwrap();
            self.free_slots.swap_remove(ind);
        } else {
            for i in self.vec.len()..id {
                self.free_slots.push(i);
            }
            self.vec.resize(id + 1, None);
        }
        self.vec[id] = Some(value);
    }

    pub fn remove(&mut self, id: usize) {
        if self.vec.len() > id {
            self.vec[id] = None;
            self.free_slots.push(id);
        }
    }

    pub fn get(&self, id: usize) -> &T {
        self.vec[id].as_ref().unwrap()
    }

    pub fn get_mut(&mut self, id: usize) -> &mut T {
        self.vec[id].as_mut().unwrap()
    }

    pub fn insert_free(&mut self, value: T) -> usize {
        if self.free_slots.is_empty() {
            let ind = self.vec.len();
            self.vec.push(Some(value));
            ind
        } else {
            let ind = self.free_slots.pop().unwrap();
            self.vec[ind] = Some(value);
            ind
        }
    }
}