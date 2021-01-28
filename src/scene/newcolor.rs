use std::sync::atomic::{AtomicUsize, Ordering};

static COUNTER: AtomicUsize = AtomicUsize::new(0);
static COLORS: [[f32; 3]; 64] = include!("newcolor_colors");

/// Get a new unique color, specified by RGB components.
pub fn new_color() -> [f32; 3] {
    COLORS[COUNTER.fetch_add(1, Ordering::Relaxed) % COLORS.len()]
}
