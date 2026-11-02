"""Validated float-math call profile; does not change equations or tolerances.
Only generated routines whose operands are real32_T are eligible. No fast-math.
Idempotent: a normal rebuild does not change an already-optimized source.
"""
from pathlib import Path
import argparse,hashlib,json,re
FILES={'src/nav/generated/nav_fusion_step.c':{'sin':'sinf','cos':'cosf','sqrt':'sqrtf','pow':'powf','fabs':'fabsf','floor':'floorf'},'src/nav/qp/mpcActiveSetSolver.c':{'fabs':'fabsf','floor':'floorf'},'src/nav/qp/norm.c':{'sqrt':'sqrtf','fabs':'fabsf'},'src/nav/qp/qpkwik.c':{'sqrt':'sqrtf','fabs':'fabsf'},'src/nav/qp/xnrm2.c':{'sqrt':'sqrtf','fabs':'fabsf'}}
def apply(root):
 rows=[]
 for name,mapping in FILES.items():
  path=root/name;before=path.read_text(encoding='utf-8');after=before;counts={}
  if 'real32_T' not in before:raise RuntimeError('Expected single-precision generated routine: '+name)
  # These files were inspected and tested with real32 operands. A new double
  # declaration is a changed model and needs its own precision review.
  if re.search(r'\b(?:double|real_T|real64_T)\s+\w+',before):raise RuntimeError('Unexpected double operand in '+name)
  for old,new in mapping.items():after,n=re.subn(r'\b'+old+r'\s*\(',new+'(',after);counts[old]=n
  if after!=before:path.write_text(after,encoding='utf-8')
  rows.append({'file':name,'replacements':counts,'changed':after!=before,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
 return {'profile':'float_math_calls_v1','equations_and_tolerances_changed':False,'fast_math':False,'files':rows}
def main():
 p=argparse.ArgumentParser();p.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[1]);p.add_argument('--report',type=Path);a=p.parse_args();result=apply(a.root)
 if a.report:a.report.parent.mkdir(parents=True,exist_ok=True);a.report.write_text(json.dumps(result,indent=2),encoding='utf-8')
 print(json.dumps({'profile':result['profile'],'changed_files':sum(x['changed'] for x in result['files']),'fast_math':False}))
if __name__=='__main__':main()
