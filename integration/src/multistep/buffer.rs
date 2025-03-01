#[derive(Clone, Debug)]
pub struct LMBuffer<T> {
    head: usize,
    data: Vec<T>,
}

impl<T> FromIterator<T> for LMBuffer<T> {
    #[inline]
    fn from_iter<I: IntoIterator<Item = T>>(iter: I) -> Self {
        let data: Vec<T> = iter.into_iter().collect();
        Self {
            head: data.len(),
            data,
        }
    }
}

impl<T> LMBuffer<T> {
    #[inline]
    pub fn front(&self) -> &T {
        &self.data[self.head]
    }

    pub fn front_mut(&mut self) -> &mut T {
        &mut self.data[self.head]
    }

    #[inline]
    pub fn rotate_left(&mut self) {
        self.head = (self.head + 1) % self.data.len();
    }

    #[inline]
    pub fn rotate_right(&mut self) {
        self.head = (self.head + self.data.len() - 1) % self.data.len();
    }

    #[inline]
    pub fn iter(&self) -> LMBufferIter<'_, T> {
        LMBufferIter {
            buffer: self,
            index: 0,
        }
    }
}

pub struct LMBufferIter<'a, T> {
    buffer: &'a LMBuffer<T>,
    index: usize,
}

impl<'a, T> Iterator for LMBufferIter<'a, T> {
    type Item = &'a T;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.buffer.data.len() {
            let item = &self.buffer.data[(self.buffer.head + self.index) % self.buffer.data.len()];
            self.index += 1;
            Some(item)
        } else {
            None
        }
    }
}
