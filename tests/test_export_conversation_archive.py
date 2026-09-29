import base64
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


class ConversationArchiveTest(unittest.TestCase):
    def test_public_only_with_redaction_questions_images_and_kst(self):
        path = Path(__file__).resolve().parents[1] / 'tools/export_conversation_archive.py'
        spec = importlib.util.spec_from_file_location('conversation_export', path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root/'config').mkdir()
            (root/'config/ntrip.local.json').write_text(json.dumps({'user':'test_user_123', 'password':'test_password_456'}))
            image_bytes = b'embedded-image-test-bytes'
            def item(payload, kind='response_item'):
                return {'timestamp':'2026-09-14T15:10:00Z','type':kind,'payload':payload}
            rows = [item({'id':'fixture-thread'},'session_meta'),
                    item({'type':'reasoning','text':'PRIVATE_REASONING'}),
                    item({'type':'message','role':'developer','content':[{'type':'input_text','text':'PRIVATE_INSTRUCTION'}]}),
                    item({'type':'message','role':'assistant','phase':'analysis','content':[{'type':'output_text','text':'PRIVATE_ANALYSIS'}]}),
                    item({'type':'message','role':'user','content':[{'type':'input_text','text':'<environment_context>AUTO_ONLY</environment_context>'}]}),
                    item({'type':'message','role':'user','content':[{'type':'input_text','text':'요청 test_password_456'},
                         {'type':'input_image','image_url':'data:image/png;base64,'+base64.b64encode(image_bytes).decode()}]}),
                    item({'type':'message','role':'assistant','phase':'final_answer','content':[{'type':'output_text','text':'공개 답변'}]}),
                    item({'type':'function_call','name':'request_user_input_async','call_id':'q1','arguments':json.dumps({'questions':[{'title':'현재 상태?'}]})}),
                    item({'type':'function_call_output','call_id':'q1','output':'질문 표시됨'}),
                    item({'type':'function_call_output','call_id':'unrelated','output':'PRIVATE_TOOL_OUTPUT'})]
            source = root/'source.jsonl'
            source.write_text(''.join(json.dumps(r)+'\n' for r in rows))
            mod.ROOT = root
            with patch.object(sys,'argv',['export','--source',str(source)]), contextlib.redirect_stdout(io.StringIO()):
                mod.main()
            output = root/'docs/conversations'
            exported = (output/'messages.jsonl').read_text()
            for hidden in ('PRIVATE_REASONING','PRIVATE_INSTRUCTION','PRIVATE_ANALYSIS','PRIVATE_TOOL_OUTPUT','AUTO_ONLY','test_password_456'):
                self.assertNotIn(hidden, exported)
            messages = [json.loads(s) for s in exported.splitlines()]
            self.assertEqual([m['kind'] for m in messages],['user','assistant','assistant_question','question_tool_result'])
            self.assertTrue(all(m['date_kst']=='2026-09-15' for m in messages))
            self.assertEqual((output/messages[0]['attachments'][0]['path']).read_bytes(),image_bytes)
            report = json.loads((output/'export_report.json').read_text())
            self.assertEqual(report['source_thread_id'],'fixture-thread')
            self.assertEqual(report['redactions']['configured_credential_matches'],1)
            self.assertEqual(report['embedded_images'],1)


if __name__ == '__main__':
    unittest.main()
