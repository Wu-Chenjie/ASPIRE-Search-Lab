"""Secondary ULog timestamp metadata; never reconstructs any sample."""
def logged_subset(expected,actual):
 expected_set=set(expected);actual_set=set(actual)
 ordered=all(a<b for a,b in zip(actual,actual[1:]))
 missing=sorted(expected_set-actual_set);last=actual[-1] if actual else None
 return dict(valid=bool(actual) and ordered and len(actual)==len(actual_set) and actual_set<=expected_set,strictly_increasing=ordered,duplicate_count=len(actual)-len(actual_set),unexpected_times=sorted(actual_set-expected_set),missing_times=missing,missing_interior_times=[t for t in missing if last is not None and t<last],missing_tail_times=[t for t in missing if last is None or t>last],records=len(actual),expected_actual_public_records=len(expected))
