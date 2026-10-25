"""Generate an explicitly staged static engineering gallery, not a new harness."""
import html,json,shutil
from pathlib import Path
from robofrontier.suite.scenes import ROOT,catalog,case_directory

def build():
    out=ROOT/'docs/generated/frontier-suite';out.mkdir(parents=True,exist_ok=True);thumbs=out/'images';thumbs.mkdir(exist_ok=True);data=catalog();cards=[]
    for c in data['cases']:
        directory=case_directory(c);views=[]
        for cam in ('global','top','global_rear'):
            source=directory/('preview-'+cam+'.png')
            if source.exists():
                target=thumbs/(c['id']+'-'+cam+'.png');shutil.copy2(source,target);views.append(f'<a href="images/{target.name}"><img loading="lazy" src="images/{target.name}" alt="{cam}"></a>')
        cards.append(f'<article><h2>{html.escape(c["id"])}</h2><p>{c["arms"]} arms · Level {c["level"]} · <strong>{html.escape(c["status"])}</strong></p><div class="views">'+''.join(views)+'</div></article>')
    page='''<!doctype html><meta charset="utf-8"><title>RoboFrontier · 36 configurations</title><style>body{background:#edf3f7;color:#17202b;font:16px system-ui;max-width:1500px;margin:32px auto;padding:0 24px}h1{font-size:32px}p{line-height:1.6}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(400px,1fr));gap:24px}article{background:#fff;border:1px solid #cad5df;border-radius:12px;padding:16px}h2{font-size:20px}.views{display:grid;grid-template-columns:2fr 1fr 1fr;gap:6px}img{width:100%;border-radius:5px}strong{color:#80521b}</style><h1>RoboFrontier · 36 configurations</h1><p>Development gallery. GENERATED and LAYOUT_CHECKED are not full task passes. Oracle results and repeatability are tracked separately. No GPT policy is running.</p><div class="grid">'''+''.join(cards)+'</div>'
    (out/'index.html').write_text(page);(out/'status.json').write_text(json.dumps(data,indent=2));print(out/'index.html')
if __name__=='__main__':build()
