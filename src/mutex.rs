use core::cell::{Cell, UnsafeCell};
use core::mem::MaybeUninit;
use core::ops::{Deref, DerefMut};
use core::ptr;
use cortex_m::interrupt::{self, CriticalSection};

pub struct Mutex<T> {
    state: Cell<u8>,
    inner: UnsafeCell<MaybeUninit<T>>,
}

const UNINITIALIZED: u8 = 0;
const INITIALIZED: u8 = 1;
const LOCKED: u8 = 2;

impl<T> Mutex<T> {
    pub const fn uninit() -> Self {
        Self {
            state: Cell::new(UNINITIALIZED),
            inner: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    pub const fn new(value: T) -> Self {
        Self {
            state: Cell::new(INITIALIZED),
            inner: UnsafeCell::new(MaybeUninit::new(value)),
        }
    }

    pub fn init(&self, value: T) {
        interrupt::free(|_| {
            if self.state.get() != UNINITIALIZED {
                panic!("Cannot initialize twice");
            }

            unsafe {
                ptr::write((*self.inner.get()).as_mut_ptr(), value);
            }

            self.state.set(INITIALIZED);
        });
    }

    pub fn lock(&self, cs: &CriticalSection) -> Guard<'_, T> {
        let _ = cs;

        if self.state.get() != INITIALIZED {
            panic!("Mutex not initialized or attempted to lock twice")
        }

        self.state.set(LOCKED);

        Guard(self)
    }
}

unsafe impl<T> Sync for Mutex<T> { }

pub struct Guard<'a, T>(&'a Mutex<T>);

impl<T> Deref for Guard<'_, T> {
    type Target = T;

    fn deref(&self) -> &T {
        unsafe { &*(&*self.0.inner.get()).as_ptr() }
    }
}

impl<T> DerefMut for Guard<'_, T> {
    fn deref_mut(&mut self) -> &mut T {
        unsafe { &mut *(&mut *self.0.inner.get()).as_mut_ptr() }
    }
}

impl<T> Drop for Guard<'_, T> {
    fn drop(&mut self) {
        self.0.state.set(INITIALIZED);
    }
}