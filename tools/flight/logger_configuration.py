"""Run-local PX4 logger setup preserving the stock etc resource view."""
import hashlib
import json
import shutil
from pathlib import Path


def install_logger_topics(work, build, topics):
    work, build, topics = Path(work), Path(build), Path(topics)
    stock = build/'etc'
    if not (stock/'init.d-posix/airframes').is_dir():
        raise ValueError('Missing verified PX4 runtime airframe resources')
    target = work/'etc'
    if target.exists() or target.is_symlink():
        raise ValueError('Refuse to replace an existing PX4 runtime etc view')
    staged = work/'.logger-etc'
    staged.mkdir()
    links = {}
    for item in stock.iterdir():
        if item.name == 'logging':
            shutil.copytree(item, staged/item.name)
        else:
            (staged/item.name).symlink_to(item.resolve(), target_is_directory=item.is_dir())
            links[item.name] = str(item.resolve())
    logging = staged/'logging'; logging.mkdir(exist_ok=True)
    shutil.copyfile(topics, logging/'logger_topics.txt')
    # Validate the exact path pattern used by rcS without launching PX4.
    if not any((staged/'init.d-posix/airframes').glob('4002_*')):
        raise ValueError('PX4 4002 autostart resource hidden by logger setup')
    staged.rename(target)
    report = dict(scope='Run-local resource links and logger topic file only; no parameters',
                  stock=str(stock), preserved_resource_links=links,
                  topics_sha256=hashlib.sha256(topics.read_bytes()).hexdigest())
    (work/'logger-install.json').write_text(json.dumps(report,indent=2)+'\n')
    return report
