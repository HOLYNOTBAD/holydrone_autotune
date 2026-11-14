"""Build the offline experiment overview from session snapshots and analysis reports."""
import json
import os
from html import escape
from pathlib import Path
from tempfile import NamedTemporaryFile
from urllib.parse import quote

import yaml

DATA = Path(__file__).resolve().parents[1] / 'data'
OUT = DATA / 'dataset'


def link(path, label=None):
    url = quote(os.path.relpath(path, OUT))
    return f'<a href="{url}">{escape(label or path.name)}</a>'


def publish(name, content):
    with NamedTemporaryFile('w', dir=OUT, delete=False, encoding='utf-8') as f:
        f.write(content)
    os.chmod(f.name, 0o644)
    os.replace(f.name, OUT / name)


def experiment(path):
    cfg = yaml.safe_load((path/'candidates.yaml').read_text())
    events = [e for e in yaml.load_all((path/'events.yaml').read_text(),
              Loader=getattr(yaml, 'CSafeLoader', yaml.SafeLoader)) if e]
    result = OUT / path.name
    report = json.loads((result/'report.json').read_text()) if (result/'report.json').exists() else {}
    pulses = report.get('pulses', [])
    axis = cfg['axis']
    status = '完整' if any(e['event_type']=='SEQUENCE_COMPLETE' for e in events) else '未完成'
    candidates = {c['id']: c for c in cfg['candidates']}
    finished = sum(e['event_type']=='TRIAL_FINISHED' for e in events)
    raw = sorted(f for f in path.iterdir() if f.is_file())
    summary = dict(session=path.name, axis=axis, status=status, candidates=len(candidates),
                   finished_trials=finished, analyzed_pulses=len(pulses),
                   raw_files=[os.path.relpath(f, OUT) for f in raw])
    rows = []
    for p in pulses:
        c = candidates[p['candidate_id']]
        fields = [p['trial_id'], p['candidate_id'], c.get('MC_'+axis.upper()+'RATE_P'),
                  c.get('MC_'+axis.upper()+'RATE_D'), p['amplitude_deg_s'], p['direction'],
                  p['tracking_rmse'], p['overshoot_percent'], p['rise_time_s']]
        rows.append('<tr>'+''.join('<td>'+escape('—' if x is None else
                    format(x,'.4g') if isinstance(x,float) else str(x))+'</td>' for x in fields)+'</tr>')
    plots = sorted(result.glob('comparison*.png'))
    outputs = [result/k for k in ('summary.csv', 'report.json') if (result/k).exists()]
    section = f'<section><h2>{escape(axis.title())} · {status}</h2><code>{escape(path.name)}</code>'
    section += f'<p>{len(candidates)} 组候选 · {finished} 个完成试次 · {len(pulses)} 个已分析脉冲</p>'
    section += '<p>实验文件　'+'　'.join(link(f) for f in raw)+'</p>'
    if outputs:
        section += '<p>分析结果　'+'　'.join(link(f) for f in outputs)+'</p>'
    heatmaps = sorted(result.glob('heatmap_*.png'))
    if heatmaps:
        section += '<h3>PD 参数热力图</h3><p>横轴 P，纵轴 D；按幅值、方向分别显示。同一实验各图共用指标色标，n 为有效/观测样本数；灰色为缺失或未观测。上升时间短不代表综合性能最佳。</p>'
        section += '<p>'+' · '.join(link(result/name) for name in ('heatmaps.html','heatmap_metrics.csv','heatmap_source.json') if (result/name).exists())+'</p>'
        selection=result/'analysis_input'/'selection.json'
        if selection.exists():
            audit=json.loads(selection.read_text())
            section += '<p>按 ULog 覆盖范围选取 %d 个试次，排除 %d 个试次。%s</p>' % (len(audit['included_trial_ids']),len(audit['excluded_trials']),link(selection,'筛选记录'))
        for f in heatmaps:
            section += '<details><summary>'+escape(f.stem)+'</summary>'+link(f.with_suffix('.pdf'),'下载 PDF')+'<img loading="lazy" src="'+quote(os.path.relpath(f,OUT))+'" alt="PD 参数热力图"></details>'
    for f in plots:
        section += '<details><summary>'+escape(f.stem)+'</summary><img loading="lazy" src="'+quote(os.path.relpath(f,OUT))+'" alt="实测角速度与目标角速度对比"></details>'
    if rows:
        section += '<details><summary>逐脉冲指标</summary><div class="scroll"><table><thead><tr>'+''.join(
            '<th>'+s+'</th>' for s in ('试次','候选','P','D','幅值 °/s','方向','RMSE rad/s','超调 %','上升时间 s')
            )+'</tr></thead><tbody>'+''.join(rows)+'</tbody></table></div></details>'
    return summary, section+'</section>'


def main():
    OUT.mkdir(exist_ok=True)
    entries, sections = [], []
    for path in sorted((DATA/'experiment').iterdir()):
        if (path/'events.yaml').is_file() and (path/'candidates.yaml').is_file():
            info, section = experiment(path)
            entries.append(info)
            sections.append(section)
    publish('catalog.json', json.dumps(entries, ensure_ascii=False, indent=2)+'\n')
    publish('overview.html', '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>角速度调参实验</title>
<style>*{box-sizing:border-box}body{background:#f1f5f6;color:#19353e;font:16px/1.55 system-ui;margin:32px auto;padding:0 18px;max-width:1100px}section{background:white;padding:18px 24px;margin:18px 0;border-left:4px solid #14847c;border-radius:6px}h2{margin:0}a{color:#076e70;overflow-wrap:anywhere}details{margin:10px 0}summary{cursor:pointer}img{max-width:100%}.scroll{overflow:auto;max-height:60vh}table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}th,td{text-align:left;padding:5px 8px;border-bottom:1px solid #ddd}th{position:sticky;top:0;background:#ebf4f3}</style>
<h1>角速度调参实验</h1>'''+f'<p>{len(entries)} 组实验 · {sum(e["finished_trials"] for e in entries)} 个完成试次 · {sum(e["analyzed_pulses"] for e in entries)} 个已分析脉冲</p>'+''.join(sections)+'</html>')


if __name__ == '__main__':
    main()
