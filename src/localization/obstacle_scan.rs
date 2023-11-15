use safe_drive::msg::common_interfaces::sensor_msgs;

pub fn get_front_distance(laser_scan:sensor_msgs::msg::LaserScan)->f32
{
    let ranges = laser_scan.ranges.as_slice();

    let array_length = ranges.len();

    let get_num = array_length / 2;

    let result = ranges.get(get_num).unwrap();

    *result
}

pub fn get_nearest(laser_scan:sensor_msgs::msg::LaserScan)->f32
{
    let ranges = laser_scan.ranges.as_slice();

    let array_len = ranges.len();

    let mut min = *ranges.get(0).unwrap();

    for u in 1..array_len
    {
        if min > *ranges.get(u).unwrap()
        {
            min = *ranges.get(u).unwrap();
        }
    }

    min
}