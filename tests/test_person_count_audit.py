"""Independent evaluation must not count duplicate track IDs as extra people."""
import json
from tools.flight.evaluate import evaluate


def test_one_person_two_matching_tracks_is_a_duplicate_not_two_people(tmp_path):
    result = dict(negative=False, wall_s=20, error=None, cleanup={}, scope='test fixture',
                  estimated_path_m=3, max_estimated_height_m=2)
    (tmp_path/'result.json').write_text(json.dumps(result))
    (tmp_path/'evaluation-truth.json').write_text(json.dumps(dict(person=[4,0,0])))
    tracks = [dict(id=i, xyz=xyz, state='confirmed') for i,xyz in
              [(1,[4,0,1]), (2,[4.1,0,1.5]), (3,[0,3,1])]]
    (tmp_path/'telemetry.jsonl').write_text(json.dumps(dict(kind='tracks',wall_s=10,data=dict(tracks=tracks)))+'\n')
    report=evaluate(tmp_path)
    assert report['confirmed_people']==1
    assert report['matched_person_tracks']==2
    assert report['duplicate_confirmed_tracks']==1
    assert report['false_confirmed_tracks']==1
    (tmp_path/'evaluation-truth.json').write_text(json.dumps(dict(person=None)))
    report=evaluate(tmp_path)
    assert report['confirmed_people']==0
    assert report['matched_person_tracks']==report['duplicate_confirmed_tracks']==0
    assert report['false_confirmed_tracks']==3
