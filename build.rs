use std::{
    f64::consts::{PI, TAU},
    fs::File,
    io::Write,
    path::Path,
};

const TABLE_SIZE: usize = 256;

fn generate_sine_lut<const N: usize>() -> [i16; N] {
    let mut table = [0i16; N];

    table.iter_mut().enumerate().for_each(|(index, value)| {
        let angle = (index as f64 / N as f64) * TAU;
        *value = (angle.sin() * i16::MAX as f64).round() as i16;
    });
    table
}

fn generate_triangle_lut<const N: usize>() -> [i16; N] {
    let amplitude = i16::MAX as f64;
    let period = N as f64;
    let mut table = [0; N];
    for (index, value) in table.iter_mut().enumerate() {
        let x = index as f64;
        *value = (4. * amplitude / period
            * ((x - period / 4.).rem_euclid(period) - period / 2.).abs()
            - amplitude) as i16;
    }
    table
}

fn generate_triangle_harmonics<const N: usize>(harmonics: u32) -> [i16; N] {
    let amplitude = i16::MAX as f64;
    let frequency = 1. / N as f64;
    let mut table = [0; N];
    for (t, value) in table.iter_mut().enumerate() {
        let t = t as f64;
        *value = (((8. / PI.powi(2))
            * (0..harmonics)
                .map(|i| {
                    let n = (2 * i + 1) as f64;
                    (-1f64).powf(i as f64) * n.powf(-2.) * (TAU * frequency * n * t).sin()
                })
                .sum::<f64>())
            * amplitude) as i16;
    }
    table
}

fn main() {
    let table = generate_sine_lut::<TABLE_SIZE>();
    let path = Path::new(&std::env::var("OUT_DIR").unwrap()).join("sine_lut.rs");
    let mut file = File::create(path).unwrap();
    writeln!(&mut file, "#[rustfmt::skip]").unwrap();
    writeln!(&mut file, "pub const SINE_LUT: [i16; {}] = [", TABLE_SIZE).unwrap();
    table.iter().for_each(|entry| {
        writeln!(&mut file, "    {},", entry).unwrap();
    });
    writeln!(&mut file, "];").unwrap();

    let table = generate_triangle_harmonics::<TABLE_SIZE>(2);
    let path = Path::new(&std::env::var("OUT_DIR").unwrap()).join("triangle_lut.rs");
    let mut file = File::create(path).unwrap();
    writeln!(&mut file, "#[rustfmt::skip]").unwrap();
    writeln!(
        &mut file,
        "pub const TRIANGLE_LUT: [i16; {}] = [",
        TABLE_SIZE
    )
    .unwrap();
    table.iter().for_each(|entry| {
        writeln!(&mut file, "    {},", entry).unwrap();
    });
    writeln!(&mut file, "];").unwrap();
}
