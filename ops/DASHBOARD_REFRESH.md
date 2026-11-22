# Refreshing the Privileged Ladder Results dashboard

Dashboard: https://claude.ai/artifact/K9qzj5LpoCTtoAQESyh37B (a Data-type artifact; its numbers come
from three attached CSVs). Do this at every check-in, after the session triage.

1. `python3 ~/workspace/dashboard_data.py /tmp/claude-1020/dash_refresh` — rebuilds runs.csv, infra.csv,
   teach.csv from the session folders (valid sessions only, via classify_sessions.py).
2. Upload each CSV, one Artifact call per file: `Artifact` publish with `url` = the dashboard,
   `file_path` = the CSV, `asset: true`. Each returns a new `/_blob/<id>`.
3. `ArtifactData get` datasets/runs, datasets/infra, datasets/teach (to read their `version`), then
   one `ArtifactData batch` of `update`s, each pinned with `if_version`, setting
   `source.url` to the new blob and `updated: {at: <now ISO UTC>, by: "Claude check-in, from the session folders on hornet"}`.
   Use `update`, never `set` (set drops fields the page writes).
4. Delete the replaced blobs (`Artifact` action `delete`, `url` + `path` = old asset id) — they
   were uploaded by Claude and nothing references them after step 3.
5. Read `data/users/me` (`check`, `runs`) if present; fix the page only for a real fault.

Current blob ids are recorded in ~/workspace/dashboard_blobs.json (update it after step 3).

Driver takeaways (dataset `takeaways`): `cd ~/workspace/ccd_priv2 && python tools/extract_debriefs.py
$(ls -d sessions/priv_v2/a[123]/route_*_privileged) --csv <dir>/debriefs.csv`, then convert to
`python3 ~/workspace/build_takeaways.py <dir>` writes takeaways.csv (A1 + ladder_plus; columns key,arm,route,attempt,ds,source,lesson,text).
