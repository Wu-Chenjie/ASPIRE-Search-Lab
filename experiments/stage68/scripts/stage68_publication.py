"""Publish a completed existing JSON string; no numerical/control or retry policy."""
def publish_ready(path,text):
    temporary=path.with_suffix('.tmp')
    with temporary.open('x') as stream:stream.write(text)
    temporary.replace(path)
