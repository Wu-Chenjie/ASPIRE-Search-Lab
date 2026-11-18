import io
import json
import struct
import tarfile
import pytest
from tools.hm3d.archive import extract


def bundle(path, name='scene/scene.glb', external=False, link=False):
    payload=json.dumps(dict(asset=dict(version='2.0'),buffers=[dict(uri='outside.bin')] if external else [])).encode()
    payload+=b' '*(-len(payload)%4)
    data=struct.pack('<4sIII4s',b'glTF',2,len(payload)+20,len(payload),b'JSON')+payload
    with tarfile.open(path,'w') as tf:
        item=tarfile.TarInfo(name);item.size=len(data)
        if link:item.type=tarfile.SYMTYPE;item.linkname='/tmp/target';item.size=0
        tf.addfile(item,None if link else io.BytesIO(data))


def test_completed_bundle_preserves_glb(tmp_path):
    p=tmp_path/'bundle.tar';bundle(p)
    r=extract(p,tmp_path/'out')
    assert r['complete'] and r['glb_count']==1
    assert (tmp_path/'out/scene/scene.glb').stat().st_size==r['members'][0]['bytes']


@pytest.mark.parametrize('name,external,link',[('../escape.glb',False,False),('/escape.glb',False,False),('scene.glb',True,False),('scene.glb',False,True)])
def test_unsafe_bundle_rejected_before_output(tmp_path,name,external,link):
    p=tmp_path/'bundle.tar';bundle(p,name,external,link)
    with pytest.raises(ValueError):extract(p,tmp_path/'out')
    assert not (tmp_path/'out').exists()


def test_partial_download_and_missing_tar_end_rejected(tmp_path):
    folder=tmp_path/'bundle.tar.download';folder.mkdir();p=folder/'bundle.tar';bundle(p)
    with pytest.raises(ValueError,match='incomplete'):extract(p,tmp_path/'out')
    truncated=tmp_path/'truncated.tar';truncated.write_bytes(p.read_bytes()[:1024])
    with pytest.raises(ValueError,match='end markers'):extract(truncated,tmp_path/'out')
