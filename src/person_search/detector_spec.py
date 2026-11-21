"""Explicit local artifact/runtime contract; a manifest never downloads weights."""
from dataclasses import dataclass,field,asdict
import hashlib
import importlib.metadata
import json
from pathlib import Path
import re
import warnings

@dataclass(frozen=True)
class DetectorSpec:
    model_sha256:str
    backend:str='pytorch'
    device:str='cpu'
    precision:str='fp32'
    runtime_versions:dict=field(default_factory=lambda:{'ultralytics':'8.3.228'})
    class_map:dict=field(default_factory=lambda:{0:'person'})
    input_size:int=640
    preprocess_contract:str='rgb8_to_bgr_ultralytics_letterbox_v1'
    mask_output_contract:str='visible_binary_original_rgb_pixels_v1'
    purpose:str='person'
    engine:dict=field(default_factory=dict)
    schema:int=1

    def __post_init__(self):
        if self.schema!=1 or not isinstance(self.model_sha256,str) or not re.fullmatch('[0-9a-fA-F]{64}',self.model_sha256):
            raise ValueError('Invalid detector schema/hash')
        object.__setattr__(self,'model_sha256',self.model_sha256.lower())
        if self.backend not in ('pytorch','tensorrt') or self.precision not in ('fp32','fp16','int8'):raise ValueError('Unsupported detector backend/precision')
        if self.device!='cpu' and not re.fullmatch('cuda:[0-9]+',self.device):raise ValueError('Explicit cpu or cuda:index required')
        if self.device=='cpu' and (self.backend!='pytorch' or self.precision!='fp32'):raise ValueError('CPU contract requires PyTorch FP32')
        if self.precision=='int8' and self.backend!='tensorrt':raise ValueError('INT8 requires a calibrated TensorRT artifact')
        if type(self.input_size) is not int or self.input_size<=0 or self.input_size%32:raise ValueError('Input size must be a positive multiple of 32')
        if self.preprocess_contract!='rgb8_to_bgr_ultralytics_letterbox_v1' or self.mask_output_contract!='visible_binary_original_rgb_pixels_v1':raise ValueError('Unsupported pixel contract')
        if not isinstance(self.runtime_versions,dict) or 'ultralytics' not in self.runtime_versions or any(
                not isinstance(k,str) or not k.strip() or not isinstance(v,str) or not v.strip()
                for k,v in self.runtime_versions.items()):
            raise ValueError('Exact runtime versions required, including Ultralytics')
        if not isinstance(self.class_map,dict) or not self.class_map:raise ValueError('Invalid class map')
        classes={}
        for key,name in self.class_map.items():
            if not (type(key) is int or isinstance(key,str) and key.isascii() and key.isdigit()):
                raise ValueError('Class IDs must be nonnegative integers')
            ident=int(key)
            if ident<0 or not isinstance(name,str):raise ValueError('Invalid class map')
            if ident in classes:raise ValueError('Duplicate class IDs')
            classes[ident]=name
        if self.purpose=='person':
            if list(classes.values())!=['person']:raise ValueError('Person detector must select one person class')
        elif self.purpose=='trace':
            if not set(classes.values())<= {'footprint','disturbed_region','fallen_material'}:raise ValueError('Unsupported trace class')
        else:raise ValueError('Invalid detector purpose')
        object.__setattr__(self,'class_map',classes)
        if self.backend=='tensorrt':
            if not {'torch','tensorrt'}<=self.runtime_versions.keys():raise ValueError('Engine requires pinned Torch and TensorRT versions')
            if not {'cuda_version','compute_capability'}<=self.engine.keys():raise ValueError('Engine build contract required')
            if any(self.engine.get(key)!=value for key,value in (
                    ('precision',self.precision),('input_size',self.input_size),('batch',1))):
                raise ValueError('Engine contract mismatch')
            if self.precision=='int8':
                calibration_hash=self.engine.get('calibration_dataset_sha256')
                if not isinstance(calibration_hash,str) or not re.fullmatch('[0-9a-fA-F]{64}',calibration_hash):
                    raise ValueError('INT8 target-domain calibration identity required')

    @classmethod
    def load(cls,path):
        return cls(**json.loads(Path(path).read_text()))

    def verify(self,path):
        path=Path(path).resolve(strict=True)
        if path.suffix!=('.engine' if self.backend=='tensorrt' else '.pt'):raise ValueError('Artifact/backend suffix mismatch')
        for package,version in self.runtime_versions.items():
            actual=importlib.metadata.version(package)
            if actual!=version:
                warnings.warn(f'Runtime version difference for {package}: expected {version}, got {actual}',RuntimeWarning)
        digest=hashlib.sha256()
        with path.open('rb') as stream:
            for chunk in iter(lambda:stream.read(1024*1024),b''):digest.update(chunk)
        if digest.hexdigest()!=self.model_sha256:raise ValueError('Model SHA256 does not match DetectorSpec')
        return path

    def verify_device(self,torch):
        if self.device=='cpu':return
        if int(self.device.split(':')[1]) >= torch.cuda.device_count():raise ValueError('Requested CUDA device unavailable')
        if self.backend=='tensorrt':
            actual='.'.join(map(str,torch.cuda.get_device_capability(self.device)))
            if actual!=self.engine['compute_capability'] or torch.version.cuda!=self.engine['cuda_version']:
                raise ValueError('TensorRT build/target device mismatch')

    def report(self):return asdict(self)
