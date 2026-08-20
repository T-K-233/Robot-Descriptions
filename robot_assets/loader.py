"""Fetch and cache robot description assets from an upstream GitHub repository."""

from pathlib import Path
import shutil
import tempfile
import threading
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen
import zipfile

# Robots live under robots/<robot>/, the franka_description layout the lite_description
# ament package also uses.
ROOT_DIR = "robots"

_FETCH_LOCK = threading.Lock()


def load(
    path: str | Path,
    repo_url: str = "https://github.com/Berkeley-Humanoids/Lite-Description",
    cache_dir: str | Path = "data/",
    timeout: float = 20.0,
) -> Path:
    """Load a robot description file from GitHub, or return it from the local cache.

    Returns ``cache_dir / path`` when it exists. Otherwise downloads the repository ZIP,
    extracts the ``robots/<robot>/`` subtree into ``cache_dir`` and returns the path.

    Args:
        path: Relative path inside the repo, such as ``"robots/lite/urdf/lite.urdf"``.
            It must start with ``robots/<robot>/`` and must not be absolute.
        repo_url: A standard ``https://github.com/owner/repo`` URL. Only github.com works.
        cache_dir: Root directory for the cached ``robots/...`` tree.
        timeout: HTTP timeout in seconds for the archive download.

    Returns:
        Absolute path to the cached file.
    """
    robot_name, relative_path = parse_upstream_asset_path(path)
    resolved_path = (Path(cache_dir) / relative_path).resolve()
    if resolved_path.exists():
        return resolved_path

    # One fetch at a time, and re-check inside the lock: a concurrent caller may have
    # populated the cache while this one waited.
    with _FETCH_LOCK:
        if not resolved_path.exists():
            fetch_robot_description(robot_name, repo_url, Path(cache_dir), timeout)

    if not resolved_path.exists():
        raise FileNotFoundError(
            f"Robot description asset is still missing after fetching "
            f"'{robot_name}': {resolved_path}"
        )
    return resolved_path


def parse_upstream_asset_path(path: str | Path) -> tuple[str, Path]:
    """Split an upstream asset path into its robot name and the path itself."""
    asset_path = Path(path).expanduser()
    if asset_path.is_absolute():
        raise ValueError(
            "Robot description asset paths must be relative upstream paths like "
            f"'robots/lite/urdf/lite.urdf', got {asset_path}"
        )
    parts = asset_path.parts
    if len(parts) < 3 or parts[0] != ROOT_DIR:
        raise ValueError(
            "Robot description asset paths must be of the form "
            f"'{ROOT_DIR}/<robot>/...', got {asset_path}"
        )
    return parts[1], asset_path


def fetch_robot_description(
    robot_name: str,
    repo_url: str,
    cache_dir: Path,
    timeout: float,
) -> None:
    """Download the repository archive and copy one robot's subtree into the cache."""
    cache_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f"robot_assets_{robot_name}_") as tmp:
        archive_path = Path(tmp) / "repo.zip"
        extracted_dir = Path(tmp) / robot_name
        download_archive(archive_path, repo_url, timeout)
        extract_robot_directory(archive_path, robot_name, extracted_dir, repo_url)
        shutil.copytree(extracted_dir, cache_dir / ROOT_DIR / robot_name, dirs_exist_ok=True)


def download_archive(destination: Path, repo_url: str, timeout: float) -> None:
    """Download the repository ZIP from the GitHub API to ``destination``."""
    request = Request(
        build_archive_url(repo_url),
        headers={
            "Accept": "application/vnd.github+json",
            "User-Agent": "robot-description-cache",
        },
    )
    try:
        with (
            urlopen(request, timeout=timeout) as response,
            destination.open("wb") as archive_file,
        ):
            shutil.copyfileobj(response, archive_file)
    except HTTPError as exc:
        raise RuntimeError(
            "Failed to download robot descriptions archive from GitHub. "
            f"HTTP {exc.code}: {exc.reason}"
        ) from exc
    except URLError as exc:
        raise RuntimeError(
            "Failed to download robot descriptions archive from GitHub. "
            f"Reason: {exc.reason}"
        ) from exc


def extract_robot_directory(
    archive_path: Path,
    robot_name: str,
    destination: Path,
    repo_url: str,
) -> None:
    """Extract one ``robots/<robot_name>/`` subtree from the archive into ``destination``.

    GitHub's zipball wraps everything in a commit-named top directory, so the ``robots``
    component is located by name rather than by a fixed depth.
    """
    found_robot = False
    with zipfile.ZipFile(archive_path) as archive:
        for info in archive.infolist():
            parts = Path(info.filename).parts
            if ROOT_DIR not in parts:
                continue
            index = parts.index(ROOT_DIR)
            if len(parts) <= index + 1 or parts[index + 1] != robot_name:
                continue

            found_robot = True
            output_path = destination / Path(*parts[index + 2:])
            if info.is_dir():
                output_path.mkdir(parents=True, exist_ok=True)
                continue
            output_path.parent.mkdir(parents=True, exist_ok=True)
            with archive.open(info) as source, output_path.open("wb") as target:
                shutil.copyfileobj(source, target)

    if not found_robot:
        raise FileNotFoundError(f"Robot '{robot_name}' was not found in {repo_url}.")


def build_archive_url(repo_url: str) -> str:
    """Derive the GitHub zipball API URL from a repository URL."""
    parsed_url = urlparse(repo_url.removesuffix(".git"))
    path_parts = [part for part in parsed_url.path.split("/") if part]
    if parsed_url.netloc != "github.com" or len(path_parts) != 2:
        raise ValueError(
            "Only standard GitHub repository URLs are supported. "
            f"Could not derive archive URL from {repo_url!r}."
        )
    owner, repo = path_parts
    return f"https://api.github.com/repos/{owner}/{repo}/zipball"
