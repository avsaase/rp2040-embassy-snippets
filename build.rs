use std::{f64::consts::TAU, fs::File, io::Write, path::Path};

const TABLE_SIZE: usize = 4096;

fn generate_lookup<const N: usize>() -> [i16; N] {
    let mut table = [0i16; N];

    table.iter_mut().enumerate().for_each(|(index, value)| {
        let angle = (index as f64 / N as f64) * TAU;
        *value = (angle.sin() * i16::MAX as f64).round() as i16;
    });
    table
}

fn main() {
    let table = generate_lookup::<TABLE_SIZE>();
    let path = Path::new(&std::env::var("OUT_DIR").unwrap()).join("sine_table.rs");
    let mut file = File::create(path).unwrap();

    writeln!(
        &mut file,
        "#[allow(clippy::excessive_precision, clippy::approx_constant)]"
    )
    .unwrap();
    writeln!(&mut file, "pub const SINE_TABLE: [i16; {}] = [", TABLE_SIZE).unwrap();
    table.iter().for_each(|entry| {
        writeln!(&mut file, "    {},", entry).unwrap();
    });
    writeln!(&mut file, "];").unwrap();
}
