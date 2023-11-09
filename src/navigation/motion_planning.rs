use safe_drive::clock::Clock;

pub fn calc_dynamic_window()
{
    let mut timer = Clock::new().unwrap();

    let now  = timer.get_now().unwrap();
}

