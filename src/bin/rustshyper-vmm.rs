use std::{env, io, process::ExitCode};

use rustshyper_vmm::vmm::{Vmm, load_config_from_args};

fn main() -> ExitCode {
    match try_main() {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) if err.kind() == io::ErrorKind::Interrupted => {
            eprintln!("{err}");
            ExitCode::SUCCESS
        }
        Err(err) => {
            eprintln!("rustshyper-vmm: {err}");
            ExitCode::FAILURE
        }
    }
}

fn try_main() -> io::Result<()> {
    let args: Vec<String> = env::args().skip(1).collect();
    let config = load_config_from_args(&args)?;
    let mut vmm = Vmm::new(&config)?;
    vmm.run()
}
