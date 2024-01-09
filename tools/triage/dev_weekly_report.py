#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2023 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

"""
Gathers information that a developer might want to include in a weekly report.
"""

import argparse
import datetime
import json
from gitea_utils import gitea_json_activities_get, gitea_json_pull_request_get
from weekly_utils import week_to_weekly_title

def main():
    args = argparse_create().parse_args()
    username = args.username

    now = datetime.datetime.now()
    monday = datetime.datetime(now.year, now.month, now.day) - datetime.timedelta(days=now.weekday())

    while True:
        if input(f"Generate for {week_to_weekly_title(monday)}? [y]/n: ") in ("", "y"):
            break
        monday -= datetime.timedelta(weeks=1)

    commit_lines = []
    touched_pull_request_ids = set()

    for weekday_index in range(7):
        day = monday + datetime.timedelta(days=weekday_index)
        day_str = day.strftime("%Y-%m-%d")
        for activity in gitea_json_activities_get(username, day_str):
            op_type = activity["op_type"]
            if op_type == "commit_repo":
                if not activity["content"]:
                    continue
                content_json = json.loads(activity["content"])
                repo_fullname = activity["repo"]["full_name"]
                if activity["ref_name"] != "refs/heads/main":
                    continue
                if ".profile" in repo_fullname:
                    continue
                for commit in content_json["Commits"]:
                    if commit["AuthorName"] != activity["act_user"]["full_name"]:
                        continue
                    commit_hash = commit["Sha1"][:10]
                    title = commit["Message"].split("\n", 1)[0]
                    commit_url = f"https://projects.blender.org/{repo_fullname}/commit/{commit_hash}"
                    commit_data = f"{title} ([{commit_hash[:7]}]({commit_url}))"
                    commit_lines.append(commit_data)
            elif op_type in ("comment_pull", "create_pull_request", "merge_pull_request"):
                pull_request_id = activity["content"].split("|")[0]
                repo_fullname = activity["repo"]["full_name"]
                touched_pull_request_ids.add((repo_fullname, pull_request_id))

    pr_lines = []

    for repo_fullname, pull_request_id in touched_pull_request_ids:
        pr_data = gitea_json_pull_request_get(repo_fullname, pull_request_id)
        if pr_data["user"]["username"].lower() != username.lower():
            continue
        if pr_data["state"] != "open":
            continue
        pr_lines.append(f"[PR #{pull_request_id}]({pr_data["html_url"]}): {pr_data["title"]}")

    commit_lines = list(sorted(commit_lines, key=lambda x: (sort_priority(x), x.lower())))
    pr_lines = list(sorted(pr_lines))

    for line in commit_lines:
        print(f"* {line}")
    for line in pr_lines:
        print(f"* {line}")

def sort_priority(line):
    line = line.lower()
    if line.startswith("fix #"):
        return 0
    if line.startswith("fix"):
        return 1
    if line.startswith("cleanup"):
        return 3
    if ":" in line:
        return 2
    return 4

def argparse_create():
    parser = argparse.ArgumentParser(description="Generate Dev Weekly Report")

    parser.add_argument(
        "--username",
        dest="username",
        type=str,
        required=True)

    return parser



if __name__ == "__main__":
    main()
