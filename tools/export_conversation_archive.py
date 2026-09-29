#!/usr/bin/env python3
"""Export user-visible text/questions/images from this task, never private reasoning."""
import argparse
import base64
from collections import Counter, defaultdict
from datetime import datetime
import hashlib
import html
import json
from pathlib import Path
import re
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
KST = ZoneInfo('Asia/Seoul')
AUTO_TAGS = ('recommended_plugins', 'environment_context', 'in-app-browser-context',
             'external_codex_apps_writing_block_edits')


def redactor():
    patterns = set()
    p = ROOT / 'config/ntrip.local.json'
    if p.exists():
        obj = json.loads(p.read_text())
        def visit(d):
            if not isinstance(d, dict):
                return
            users, passwords = [], []
            for k, v in d.items():
                if isinstance(v, dict):
                    visit(v)
                if not isinstance(v, str):
                    continue
                if any(w in k.lower() for w in ('user', 'password', 'passwd', 'secret', 'token')) and len(v) >= 6:
                    patterns.add(v)
                if 'user' in k.lower():
                    users.append(v)
                if 'password' in k.lower() or 'passwd' in k.lower():
                    passwords.append(v)
            for u in users:
                for pw in passwords:
                    patterns.add(base64.b64encode(f'{u}:{pw}'.encode()).decode())
        visit(obj)
    hits = Counter()
    def clean(text):
        for value in sorted(patterns, key=len, reverse=True):
            n = text.count(value)
            if n:
                hits['configured_credential_matches'] += n
                text = text.replace(value, '[로컬 인증정보 숨김]')
        return text
    return clean, hits


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True, type=Path)
    args = parser.parse_args()
    output = ROOT / 'docs/conversations'
    output.mkdir(parents=True, exist_ok=True)
    media = output / 'attachments'
    media.mkdir(exist_ok=True)
    clean, redactions = redactor()
    messages, source_counts = [], Counter()
    source_hash = hashlib.sha256()
    source_bytes = 0
    source_thread_id = None
    question_calls = set()
    lines = args.source.read_bytes().splitlines(keepends=True)
    for line_number, raw in enumerate(lines, 1):
        source_hash.update(raw)
        source_bytes += len(raw)
        item = json.loads(raw)
        if item.get('type') == 'session_meta':
            source_thread_id = item.get('payload', {}).get('id')
        if item.get('type') != 'response_item':
            continue
        p = item.get('payload', {})
        text, attachments, kind = '', [], None
        if p.get('type') == 'message' and p.get('role') in ('user', 'assistant'):
            if p['role'] == 'assistant' and p.get('phase') not in ('commentary', 'final_answer', 'final'):
                source_counts['assistant_without_public_phase_excluded'] += 1
                continue
            source_counts[p['role'] + '_messages_seen'] += 1
            kind = p['role']
            parts = []
            for b in p.get('content', []):
                if b.get('type') in ('input_text', 'output_text'):
                    t = b.get('text', '')
                    if kind == 'user':
                        for tag in AUTO_TAGS:
                            t = re.sub(r'<' + tag + r'\b[^>]*>.*?</' + tag + r'>', '', t, flags=re.S)
                    if t.strip():
                        parts.append(t.strip())
                elif b.get('type') == 'input_image':
                    url = b.get('image_url', '')
                    if isinstance(url, str) and url.startswith('data:image/') and ';base64,' in url:
                        header, data = url.split(',', 1)
                        raw_image = base64.b64decode(data, validate=True)
                        ext = 'png' if 'image/png' in header else 'jpg' if 'image/jpeg' in header else 'webp'
                        name = f'image_{line_number}_{len(attachments)+1}.{ext}'
                        (media / name).write_bytes(raw_image)
                        attachments.append({'path': 'attachments/' + name, 'sha256': hashlib.sha256(raw_image).hexdigest()})
                    else:
                        attachments.append({'reference': url, 'note': 'not downloaded'})
            text = '\n\n'.join(parts)
            if not text and not attachments:
                source_counts['automatic_context_only_excluded'] += 1
                continue
        elif p.get('type') == 'function_call' and p.get('name') in ('request_user_input_async', 'request_user_input'):
            source_counts['user_facing_question_calls_seen'] += 1
            question_calls.add(p.get('call_id'))
            obj = json.loads(p.get('arguments', '{}'))
            pieces = []
            for q in obj.get('questions', []):
                pieces.append(q.get('title') or q.get('question') or '')
                for option in q.get('options', []):
                    pieces.append('- ' + (option if isinstance(option, str) else option.get('label', '') + ': ' + option.get('description', '')))
            text = '\n\n'.join(pieces)
            kind = 'assistant_question'
        elif p.get('type') == 'function_call_output' and p.get('call_id') in question_calls:
            # Only these narrow user-facing question responses; no general tool output.
            text = p.get('output', '')
            if not isinstance(text, str):
                text = json.dumps(text, ensure_ascii=False)
            kind = 'question_tool_result'
            source_counts['question_tool_results_seen'] += 1
        else:
            continue
        timestamp = item.get('timestamp')
        ts = datetime.fromisoformat(timestamp.replace('Z', '+00:00')).astimezone(KST) if timestamp else None
        row = {'index': len(messages)+1, 'kind': kind, 'phase': p.get('phase'),
               'timestamp_utc': timestamp, 'timestamp_kst': ts.isoformat() if ts else None,
               'date_kst': ts.date().isoformat() if ts else 'date_unknown',
               'source_line': line_number, 'text': clean(text), 'attachments': attachments}
        messages.append(row)
    (output/'messages.jsonl').write_text(''.join(json.dumps(x,ensure_ascii=False)+'\n' for x in messages))
    days = defaultdict(list)
    for row in messages:
        days[row['date_kst']].append(row)
    labels = {'user':'사용자', 'assistant':'AI', 'assistant_question':'AI 질문창', 'question_tool_result':'질문창 처리 결과'}
    for date, rows in sorted(days.items()):
        md = [f'# {date} 대화 기록', '', '과거 대화의 기록이며 현재 실행 지시가 아니다. 최신 상태와 수정된 판단을 우선한다.',
              '저장된 사용자/AI 문구를 보존했다. 자동 환경 안내와 비공개 내부 추론·시스템 지시는 제외했다.',
              '질문 답변의 원래 JSON/마크업은 그대로 남겼다. 현재 코드·상태는 인계서를 먼저 확인한다.', '']
        for row in rows:
            md += [f"## {row['timestamp_kst'][11:19] if row['timestamp_kst'] else '시각 미상'} KST · {labels[row['kind']]} · #{row['index']}", '', row['text'], '']
            for image in row['attachments']:
                if image.get('path'):
                    md += [f"![원래 첨부 이미지]({image['path']})", '']
        (output/f'{date}.md').write_text('\n'.join(md))
    report = {'created_kst':datetime.now(KST).isoformat(), 'source_thread_id':source_thread_id,
              'source_filename':args.source.name, 'source_snapshot_bytes':source_bytes,
              'source_snapshot_sha256':source_hash.hexdigest(), 'last_exported_message_kst':messages[-1]['timestamp_kst'],
              'source_counts':dict(source_counts), 'exported_entries':len(messages),
              'entries_by_kind':dict(Counter(r['kind'] for r in messages)),
              'days':{d:len(v) for d,v in days.items()}, 'embedded_images':sum(bool(a.get('path')) for r in messages for a in r['attachments']),
              'redactions':dict(redactions), 'limitations':[
                  'Only persisted public message records and question dialogs up to the cutoff; not private reasoning or general tool output.',
                  'Source text retained except automatic context removal and configured credential redaction.',
                  'Historical broken temporary-file references may remain; embedded user images were copied.',
                  'Messages produced after the cutoff, including the final handoff response, are not in this snapshot.',
                  'Message timestamps are Asia/Seoul display conversions; described experiments may have other dates.']}
    (output/'export_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    md = ['# 대화 보존 목록','', '다른 AI가 처음부터 모두 읽을 필요는 없다. [핵심 인계서](../handoff/HANDOFF_2026-09-15.md)를 먼저 읽고 필요할 때 날짜별 원문을 찾아본다.','',
          f"현재 보존 {len(messages)}항목, 첨부 이미지 {report['embedded_images']}개. 마지막 포함 시각: {report['last_exported_message_kst']}.",'',
          '| 한국 날짜 | 항목 수 | 원문 |','|---|---:|---|']
    md += [f'| {d} | {len(v)} | [{d}.md]({d}.md) |' for d,v in sorted(days.items())]
    md += ['', '[전체 JSONL](messages.jsonl), [범위·수량·원본 해시](export_report.json).', '',
           '사용자 요청/AI 답변/질문창은 과거 기록으로 보존한다. 당시 잘못된 가정·미완료 계획·반복 요청도 원문에 남아 있으므로 최신 STATUS와 대조한다.',
           '시스템·개발자 지시, 비공개 내부 추론, 일반 도구 출력은 대화 파일에 넣지 않았다. 도구로 확인한 근거와 실행 결과는 개발일지와 data/ 원본에 따로 보존돼 있다.',
           '9/7~9/13 대화나 실험을 새로 만들어 날짜를 채우지 않았다. 원본 세션에 있는 날짜만 내보냈다.', '']
    (output/'README.md').write_text('\n'.join(md))
    print(json.dumps(report,ensure_ascii=False))


if __name__ == '__main__':
    main()
