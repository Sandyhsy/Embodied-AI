"""
download_so_arm100.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Downloads the TRS SO-ARM100 MJCF model from:
  https://github.com/chernyadev/mujoco_menagerie/tree/add-so-arm100/trs_so_arm100

Run:
    python download_so_arm100.py

Output folder:  models/trs_so_arm100/
"""

import os
import sys
import urllib.request
import json

# ─── Config ───────────────────────────────────────────────────────────────────
REPO_OWNER  = "chernyadev"
REPO_NAME   = "mujoco_menagerie"
BRANCH      = "add-so-arm100"
FOLDER      = "trs_so_arm100"
LOCAL_DIR   = os.path.join("models", FOLDER)

RAW_BASE    = f"https://raw.githubusercontent.com/{REPO_OWNER}/{REPO_NAME}/{BRANCH}/{FOLDER}"
API_BASE    = f"https://api.github.com/repos/{REPO_OWNER}/{REPO_NAME}/contents/{FOLDER}?ref={BRANCH}"

# ─── Helpers ──────────────────────────────────────────────────────────────────
def download(url: str, dest: str) -> bool:
    os.makedirs(os.path.dirname(dest) if os.path.dirname(dest) else ".", exist_ok=True)
    try:
        headers = {"User-Agent": "Mozilla/5.0"}
        req  = urllib.request.Request(url, headers=headers)
        with urllib.request.urlopen(req, timeout=20) as r, open(dest, "wb") as f:
            f.write(r.read())
        size = os.path.getsize(dest)
        print(f"  ✓  {dest}  ({size:,} bytes)")
        return True
    except Exception as e:
        print(f"  ✗  {dest}  — {e}")
        return False


def list_github_dir(api_url: str) -> list:
    """Returns list of {name, type, download_url, path} dicts."""
    try:
        req = urllib.request.Request(api_url, headers={"User-Agent": "Mozilla/5.0"})
        with urllib.request.urlopen(req, timeout=20) as r:
            return json.loads(r.read().decode())
    except Exception as e:
        print(f"  GitHub API error: {e}")
        return []


def download_dir(api_url: str, raw_prefix: str, local_prefix: str):
    """Recursively download a GitHub directory."""
    items = list_github_dir(api_url)
    if not items:
        return

    for item in items:
        name  = item["name"]
        itype = item["type"]
        local = os.path.join(local_prefix, name)

        if itype == "file":
            url = item.get("download_url") or f"{raw_prefix}/{name}"
            download(url, local)

        elif itype == "dir":
            sub_api = item["url"]
            download_dir(sub_api, f"{raw_prefix}/{name}", local)


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    print()
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("  SO-ARM100 MuJoCo Menagerie — Model Downloader")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print(f"  Source : github.com/{REPO_OWNER}/{REPO_NAME} @ {BRANCH}")
    print(f"  Folder : {FOLDER}/")
    print(f"  Output : {LOCAL_DIR}/")
    print()

    os.makedirs(LOCAL_DIR, exist_ok=True)

    # ── Attempt recursive GitHub API download ─────────────────────────────────
    print("▶  Fetching file list from GitHub API...")
    items = list_github_dir(API_BASE)

    if items:
        print(f"   Found {len(items)} items at top level.\n")
        download_dir(API_BASE, RAW_BASE, LOCAL_DIR)

    else:
        # ── Fallback: known file list (Menagerie standard layout) ─────────────
        print("   API unavailable — falling back to known file list.\n")

        KNOWN_FILES = [
            "scene.xml",
            "so_arm100.xml",
            "LICENSE",
            "README.md",
            "assets/base.stl",
            "assets/link1.stl",
            "assets/link2.stl",
            "assets/link3.stl",
            "assets/link4.stl",
            "assets/link5.stl",
            "assets/link6.stl",
            "assets/gripper_base.stl",
            "assets/finger_left.stl",
            "assets/finger_right.stl",
        ]

        for rel in KNOWN_FILES:
            url  = f"{RAW_BASE}/{rel}"
            dest = os.path.join(LOCAL_DIR, rel.replace("/", os.sep))
            download(url, dest)

    # ── Verify key files ──────────────────────────────────────────────────────
    print()
    print("▶  Verifying downloaded files...")
    for fname in ["scene.xml", "so_arm100.xml"]:
        fpath = os.path.join(LOCAL_DIR, fname)
        if os.path.exists(fpath) and os.path.getsize(fpath) > 0:
            print(f"  ✓  {fname}")
        else:
            print(f"  ✗  {fname} — missing or empty!")

    # ── Quick MuJoCo load test ────────────────────────────────────────────────
    print()
    print("▶  Testing model load in MuJoCo...")
    try:
        import mujoco

        # Try scene first, then robot XML directly
        for candidate in ["scene.xml", "so_arm100.xml"]:
            xml_path = os.path.join(LOCAL_DIR, candidate)
            if os.path.exists(xml_path):
                try:
                    model = mujoco.MjModel.from_xml_path(xml_path)
                    data  = mujoco.MjData(model)
                    mujoco.mj_forward(model, data)

                    print(f"  ✅  Loaded {candidate}")
                    print(f"      nq={model.nq}  nv={model.nv}  nbody={model.nbody}")

                    # Print joint names
                    print("      Joints:")
                    for i in range(model.njnt):
                        j = model.joint(i)
                        print(f"        [{i}] {j.name}")
                    break
                except Exception as e:
                    print(f"  ✗  {candidate}: {e}")

    except ImportError:
        print("  (MuJoCo not installed yet — run: pip install mujoco)")

    # ── Print next steps ─────────────────────────────────────────────────────
    print()
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print("  Done! Next steps:")
    print()
    print("  • Jupyter notebook:")
    print("      jupyter lab 01_mujoco_basics.ipynb")
    print()
    print("  • Keyboard simulation:")
    print("      python so_arm101_keyboard_sim.py")
    print()
    print("  • View model directly:")
    scene = os.path.join(LOCAL_DIR, "scene.xml")
    print(f"      python -m mujoco.viewer --mjcf {scene}")
    print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
    print()


if __name__ == "__main__":
    main()
