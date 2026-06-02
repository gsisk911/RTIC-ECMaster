use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    fs::copy("memory.x", out_dir.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");

    emit_firmware_tag();
}

/// Captures a build-provenance tag so the exact running build can be confirmed
/// over USB serial. Resolves to `v<pkg>-g<short-sha>[-dirty]` inside a git
/// checkout, and degrades to `v<pkg>-nogit` when git is missing, the tree is
/// not a repository, or no commit exists yet.
fn emit_firmware_tag() {
    let pkg_version = env::var("CARGO_PKG_VERSION").unwrap_or_else(|_| "0.0.0".to_string());

    let tag = match git_short_sha() {
        Some(sha) => {
            let dirty = if git_tree_is_dirty() { "-dirty" } else { "" };
            format!("v{pkg_version}-g{sha}{dirty}")
        }
        None => format!("v{pkg_version}-nogit"),
    };

    // Refresh the tag when the checked-out commit or working-tree index moves,
    // without forcing a rerun on trees that have no git metadata.
    for path in [".git/HEAD", ".git/index"] {
        if Path::new(path).exists() {
            println!("cargo:rerun-if-changed={path}");
        }
    }

    println!("cargo:rustc-env=FW_TAG={tag}");
}

/// Returns the abbreviated commit hash, or `None` when git cannot resolve one.
fn git_short_sha() -> Option<String> {
    let output = Command::new("git")
        .args(["rev-parse", "--short=8", "HEAD"])
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let sha = String::from_utf8(output.stdout).ok()?.trim().to_string();
    if sha.is_empty() {
        None
    } else {
        Some(sha)
    }
}

/// Returns true when the working tree has uncommitted changes.
fn git_tree_is_dirty() -> bool {
    Command::new("git")
        .args(["status", "--porcelain"])
        .output()
        .map(|out| out.status.success() && !out.stdout.is_empty())
        .unwrap_or(false)
}
