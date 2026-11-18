"""Bounded apparent-size sampling; never interpret an unreadable tree as empty."""
import os
import stat as stat_module
from pathlib import Path

class StorageSampleError(RuntimeError):
    pass


def tree_bytes(root, *, attempts=3):
    """Retry a complete scan for vanished atomic .tmp files only.

    Successful size includes directory metadata, like du -sb. Hard links may be
    counted more than once (conservative). This is not atomic filesystem accounting.
    Missing roots/directories, ordinary files and permission/I/O errors fail closed.
    Continuous temporary-file churn also fails closed after the bounded attempts.
    """
    if attempts < 1:
        raise ValueError('attempts must be positive')
    root = Path(root)
    for attempt in range(attempts):
        try:
            total = root.lstat().st_size
            pending = [root]
            while pending:
                directory = pending.pop()
                with os.scandir(directory) as entries:
                    for entry in entries:
                        try:
                            stat = entry.stat(follow_symlinks=False)
                        except FileNotFoundError:
                            if entry.name.endswith('.tmp'):
                                raise _TransientAtomicReplacement(entry.path)
                            raise
                        total += stat.st_size
                        if stat_module.S_ISDIR(stat.st_mode):
                            pending.append(Path(entry.path))
            return {'bytes': total, 'attempts': attempt + 1}
        except _TransientAtomicReplacement as ex:
            if attempt + 1 == attempts:
                raise StorageSampleError('Persistent atomic temporary-file churn: ' + str(ex)) from ex
        except OSError as ex:
            raise StorageSampleError('Storage snapshot unavailable: ' + str(ex)) from ex


class _TransientAtomicReplacement(Exception):
    pass
