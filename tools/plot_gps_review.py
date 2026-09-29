#!/usr/bin/env python3
"""Plot original GGA coordinates; split lines on gaps, fix changes and jumps."""
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
import math
import statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[1]
SOURCES = [
    ('1회차', 'rtk_20260906_130102_a4134681'),
    ('2회차', 'rtk_20260906_135920_c27faf0a'),
    ('3회차', 'rtk_20260906_142653_873a7ebb'),
    ('4회차', 'rtk_20260906_143213_5de9e922'),
    ('5회차', 'rtk_20260906_153320_08e08adc'),
    ('GPS 단독 / 걷기 요청 후', 'rtk_20260906_155834_1ae690e3'),
]


def main():
    out = ROOT / 'docs/analysis/gps_review_2026-09-14'
    out.mkdir(parents=True, exist_ok=True)
    font = FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams['font.family'] = font.get_name()
    plt.rcParams['axes.unicode_minus'] = False
    lat0, lon0 = 37.2888, 127.1072
    def xy(r):
        return ((r['longitude_deg']-lon0)*111195*math.cos(math.radians(lat0)),
                (r['latitude_deg']-lat0)*111195)
    fig, axes = plt.subplots(2, 3, figsize=(13, 8), sharex=True, sharey=True)
    reports = []
    for ax, (label, name) in zip(axes.flat, SOURCES):
        p = ROOT/'data'/name/'gga.jsonl'
        raw = p.read_bytes()
        rows = [json.loads(line) for line in raw.splitlines()]
        valid = [r for r in rows if r.get('quality_code', 0)>0
                 and r.get('latitude_deg') is not None and r.get('longitude_deg') is not None]
        fixed = [r for r in valid if r['quality_code']==4]
        jumps = []
        for a, b in zip(rows, rows[1:]):
            if any(r.get('quality_code',0)==0 or r.get('latitude_deg') is None
                   or r.get('longitude_deg') is None for r in (a,b)):
                continue
            dt = b['elapsed_s']-a['elapsed_s']
            x1,y1=xy(a); x2,y2=xy(b); distance=math.hypot(x2-x1,y2-y1)
            if dt>0 and distance/dt>3:
                jumps.append({'host_utc':b['host_utc'], 'distance_m':distance,
                              'dt_s':dt, 'quality_before':a['quality_code'], 'quality_after':b['quality_code']})
            if 0<dt<=2.5 and distance/dt<=3 and a['quality_code']==b['quality_code']:
                ax.plot([x1,x2],[y1,y2],color='#16896b' if b['quality_code']==4 else '#e29823',linewidth=1,alpha=.6)
        for group,color in [(fixed,'#16896b'),([r for r in valid if r['quality_code']!=4],'#e29823')]:
            if group:
                xx,yy=zip(*(xy(r) for r in group));ax.scatter(xx,yy,s=5,c=color,zorder=3)
        ax.scatter(*xy(valid[0]),marker='^',s=70,color='#2563eb',zorder=5)
        ax.scatter(*xy(valid[-1]),marker='X',s=65,color='#be123c',zorder=5)
        ax.set_title(f'{label}\nFIX {len(fixed)}/{len(rows)} ({len(fixed)/len(rows):.1%})',fontsize=11)
        ax.set_aspect('equal');ax.set_xlim(-65,65);ax.set_ylim(-60,50);ax.grid(alpha=.2)
        if ax in axes[1]:
            ax.set_xlabel('동쪽 거리 (m)')
        ax.set_ylabel('북쪽 거리 (m)')
        reports.append({'label':label,'source':str(p.relative_to(ROOT)),'source_sha256':hashlib.sha256(raw).hexdigest(),
                        'samples':len(rows),'valid_coordinates':len(valid),'fixed':len(fixed),
                        'first_valid_lat_lon':[valid[0]['latitude_deg'],valid[0]['longitude_deg']],
                        'last_valid_lat_lon':[valid[-1]['latitude_deg'],valid[-1]['longitude_deg']],
                        'median_fixed_lat_lon':[statistics.median(r[k] for r in fixed) for k in ('latitude_deg','longitude_deg')],
                        'jumps_over_3_mps':jumps})
    fig.suptitle('9월 6일 실제 GPS 기록 · 회차별 위치와 품질',fontsize=19,y=.98)
    handles=[Line2D([0],[0],color='#16896b',marker='.',label='RTK FIX'),
             Line2D([0],[0],color='#e29823',marker='.',label='FLOAT / DGPS / GPS'),
             Line2D([0],[0],color='#2563eb',marker='^',linestyle='',label='첫 유효 위치'),
             Line2D([0],[0],color='#be123c',marker='X',linestyle='',label='마지막 유효 위치')]
    fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.5,.938),ncol=4,frameon=False)
    fig.text(.5,.018,'원점: 북위 37.288800°, 동경 127.107200° · 위=북쪽 · 배경지도 없음 · FIX도 절대오차 검증은 아님\n2.5초 초과 공백·3m/s 초과 위치변화·품질 변경 지점은 선을 연결하지 않음',ha='center',fontsize=10)
    fig.subplots_adjust(left=.065,right=.985,bottom=.14,top=.80,hspace=.40,wspace=.25)
    fig.savefig(out/'gps_tracks.png',dpi=160);plt.close(fig)
    report={'created_kst':datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),'sessions':reports,
            'note':'3m/s is a review heuristic, not a calibrated rejection criterion. No source data changed.'}
    (out/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps([{'label':r['label'],'fixed':r['fixed'],'samples':r['samples'],'jump_count':len(r['jumps_over_3_mps'])} for r in reports],ensure_ascii=False))


if __name__=='__main__':
    main()
