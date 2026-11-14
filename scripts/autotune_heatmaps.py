#!/usr/bin/env python3
"""Generate offline PD heatmaps from an archived session and analyzed report.

Example: python scripts/autotune_heatmaps.py --session data/experiment/<session> --report data/dataset/<session>/report.json --output data/dataset/<session>
"""
import argparse
import csv
import json
import subprocess
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT = Path(__file__).resolve().parents[1]
METRICS = [('tracking_rmse', 'RMSE (rad/s)', 1), ('overshoot_percent', 'Overshoot (%)', 1), ('rise_time_s', 'Rise time (ms)', 1000)]
def run(*args):
    return subprocess.check_output(args, cwd=ROOT, text=True)

def plot(session, report, dest):
    config=yaml.safe_load((session/'candidates.yaml').read_text())
    axis=config['axis']; prefix='MC_'+axis.upper()+'RATE_'
    candidates={c['id']:c for c in config['candidates']}
    events=[e for e in yaml.safe_load_all((session/'events.yaml').read_text()) if e]
    starts={e['trial_id']:e for e in events if e.get('event_type')=='TRIAL_START'}
    data=json.loads(report.read_text())
    rows=[]
    for pulse in data['pulses']:
        event=starts[pulse['trial_id']]
        actual=event.get('readback_parameters') or {}
        if not all(prefix+t in actual for t in ('P','I','D','FF','K')):
            raise ValueError('Missing actual parameter readback for trial '+str(pulse['trial_id']))
        # Check archived candidate against actual FCU readback; use nominal grid labels only after matching.
        c=candidates[pulse['candidate_id']]
        if any(not np.isclose(actual[prefix+t],c[prefix+t],rtol=1e-5,atol=1e-8) for t in ('P','I','D','FF','K')):
            raise ValueError('Candidate/readback mismatch: '+pulse['candidate_id'])
        rows.append(dict(pulse, P=c[prefix+'P'], D=c[prefix+'D'], I=c[prefix+'I'], FF=c[prefix+'FF'], K=c[prefix+'K']))
    fixed={(r['I'],r['FF'],r['K']) for r in rows}
    if len(fixed)!=1: raise ValueError('I/FF/K varied; cannot combine in one PD grid')
    ps=sorted({c[prefix+'P'] for c in candidates.values()}); ds=sorted({c[prefix+'D'] for c in candidates.values()})
    dest.mkdir(parents=True,exist_ok=True)
    (dest/'heatmap_source.json').write_text(json.dumps(dict(session=str(session), report=str(report), commit=run('git','rev-parse','HEAD').strip(), fixed_I_FF_K=list(fixed)[0], excluded_trials=data.get('excluded_trials',[])),indent=2))
    records=[]; pictures=[]
    for amp in sorted({r['amplitude_deg_s'] for r in rows}):
        # Same metric scale across all amplitudes and directions within this session.
        for direction in ('positive','negative'):
            fig,axs=plt.subplots(1,3,figsize=(17,5),constrained_layout=True)
            for ax,(metric,label,factor) in zip(axs,METRICS):
                z=np.full((len(ds),len(ps)),np.nan); labels={}
                allvals=[r[metric]*factor for r in rows if r.get(metric) is not None and np.isfinite(r[metric])]
                for iy,d in enumerate(ds):
                    for ix,p in enumerate(ps):
                        selected=[r for r in rows if r['P']==p and r['D']==d and r['direction']==direction and r['amplitude_deg_s']==amp]
                        vals=[r[metric]*factor for r in selected if r.get(metric) is not None and np.isfinite(r[metric])]
                        mean=float(np.mean(vals)) if vals else None
                        sd=float(np.std(vals,ddof=1)) if len(vals)>1 else None
                        records.append(dict(axis=axis,amplitude_deg_s=amp,direction=direction,P=p,D=d,metric=label,mean=mean,std=sd,valid_count=len(vals),pulse_count=len(selected)))
                        if mean is not None:z[iy,ix]=mean
                        labels[iy,ix]=('%.3g\nn=%d/%d'%(mean,len(vals),len(selected))) if vals else ('missing' if not selected else 'unobserved')
                cmap=plt.get_cmap('RdYlGn_r').copy();cmap.set_bad('#dddddd')
                vmin=min(allvals) if allvals else 0
                vmax=max(allvals) if allvals else 1
                im=ax.imshow(np.ma.masked_invalid(z),origin='lower',aspect='auto',cmap=cmap,vmin=vmin,vmax=max(vmax,vmin+1e-9))
                ax.set_xticks(range(len(ps)),['%g'%p for p in ps]);ax.set_yticks(range(len(ds)),['%g'%d for d in ds]);ax.set_xlabel('P');ax.set_ylabel('D');ax.set_title(label)
                for (iy,ix),labeltext in labels.items():ax.text(ix,iy,labeltext,ha='center',va='center',fontsize=8)
                fig.colorbar(im,ax=ax,shrink=.8)
            i,ff,k=next(iter(fixed))
            fig.suptitle('%s | %g deg/s | %s | I=%g, FF=%g, K=%g\nMean per cell; n=valid/observed pulses; gray=missing/unobserved; shorter rise is not always better'%(axis,amp,direction,i,ff,k),fontsize=11)
            name='heatmap_%s_%g_%s'%(axis,amp,direction)
            fig.savefig(dest/(name+'.png'),dpi=180);fig.savefig(dest/(name+'.pdf'));plt.close(fig);pictures.append(name+'.png')
    with (dest/'heatmap_metrics.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    links = ''.join('<li><a href="%s">%s</a> · <a href="%s">PDF</a></li>' % (name, name, name.replace('.png','.pdf')) for name in pictures)
    images = ''.join('<h2>%s</h2><img loading="lazy" src="%s" style="max-width:100%%">' % (name,name) for name in pictures)
    (dest/'heatmaps.html').write_text('<!doctype html><meta charset="utf-8"><title>PD heatmaps</title><h1>PD heatmaps</h1><p>Axes: P horizontal, D vertical. Same color scale across amplitudes and directions in this session. Gray means missing or unobserved. n is valid / observed pulse count. Standard deviation is undefined for one sample. Smaller rise time alone is not a controller ranking.</p><a href="heatmap_metrics.csv">Metrics CSV</a> · <a href="heatmap_source.json">Source</a><ul>'+links+'</ul>'+images)
    return pictures

if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,required=True)
    p.add_argument('--report',type=Path)
    p.add_argument('--output',type=Path,help='default: data/dataset/<session name>')
    a=p.parse_args()
    session=a.session.resolve()
    output=(a.output or ROOT/'data/dataset'/session.name).resolve()
    report=(a.report or output/'report.json').resolve()
    print('Generated:',plot(session,report,output))
    subprocess.run([__import__('sys').executable,str(ROOT/'scripts/autotune_catalog.py')],check=True)
