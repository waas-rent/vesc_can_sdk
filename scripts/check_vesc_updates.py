#!/usr/bin/env python3
"""
VESC Firmware Update Checker

This script monitors the VESC firmware repository (https://github.com/vedderb/bldc)
for changes that might affect the VESC CAN SDK. It uses OpenAI to analyze commits
and determine if any changes require updates to the SDK.

Usage:
    python scripts/check_vesc_updates.py

Requirements:
    - openai
    - requests
    - python-dotenv (for API key management)
"""

import os
import json
import requests
import time
from typing import List, Dict, Optional
import argparse
from datetime import datetime

try:
    import openai
    from dotenv import load_dotenv
except ImportError as e:
    print(f"Missing required package: {e}")
    print("Please install required packages:")
    print("pip install openai requests python-dotenv")
    exit(1)

# Configuration
VESC_REPO = "vedderb/bldc"
GITHUB_API_BASE = "https://api.github.com"
COMMIT_CHECK_FILE = "last_checked_commit.txt"
MAX_COMMITS_TO_CHECK = 5
OUTPUT_DIR = "output"

# Load environment variables
load_dotenv()


class VESCUpdateChecker:
    def __init__(self):
        self.api_key = os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            print("Error: OPENAI_API_KEY environment variable not set")
            print(
                "Please set your OpenAI API key in a .env file or environment variable"
            )
            exit(1)

        # GitHub token for higher rate limits
        self.github_token = os.getenv("GITHUB_TOKEN")
        if not self.github_token:
            print(
                "Warning: GITHUB_TOKEN not set. Using unauthenticated requests (60 req/hour limit)"
            )
            print(
                "For higher limits, set GITHUB_TOKEN in .env file or environment variable"
            )

        self.last_checked_commit = self.load_last_checked_commit()

    def load_last_checked_commit(self) -> Optional[str]:
        """Load the last checked commit hash from file"""
        try:
            if os.path.exists(COMMIT_CHECK_FILE):
                with open(COMMIT_CHECK_FILE, "r") as f:
                    return f.read().strip()
        except Exception as e:
            print(f"Warning: Could not load last checked commit: {e}")
        return None

    def save_last_checked_commit(self, commit_hash: str):
        """Save the last checked commit hash to file"""
        try:
            with open(COMMIT_CHECK_FILE, "w") as f:
                f.write(commit_hash)
        except Exception as e:
            print(f"Warning: Could not save last checked commit: {e}")

    def get_github_headers(self):
        """Get headers for GitHub API requests"""
        headers = {
            "Accept": "application/vnd.github.v3+json",
            "User-Agent": "VESC-CAN-SDK-Update-Checker",
        }
        if self.github_token:
            headers["Authorization"] = f"token {self.github_token}"
        return headers

    def get_recent_commits(self) -> List[Dict]:
        """Fetch recent commits from the VESC repository"""
        url = f"{GITHUB_API_BASE}/repos/{VESC_REPO}/commits"
        params = {"per_page": MAX_COMMITS_TO_CHECK, "sha": "master"}

        try:
            response = requests.get(
                url, params=params, headers=self.get_github_headers()
            )
            response.raise_for_status()
            commits = response.json()

            # Filter commits if we have a last checked commit
            if self.last_checked_commit:
                filtered_commits = []
                for commit in commits:
                    if commit["sha"] == self.last_checked_commit:
                        break
                    filtered_commits.append(commit)
                return filtered_commits

            return commits

        except requests.RequestException as e:
            print(f"Error fetching commits: {e}")
            return []

    def get_commit_details(self, commit_sha: str) -> Optional[Dict]:
        """Get detailed information about a specific commit"""
        url = f"{GITHUB_API_BASE}/repos/{VESC_REPO}/commits/{commit_sha}"

        max_retries = 3
        for attempt in range(max_retries):
            try:
                response = requests.get(url, headers=self.get_github_headers())

                if (
                    response.status_code == 403
                    and "rate limit" in response.text.lower()
                ):
                    # Rate limit hit
                    reset_time = response.headers.get("X-RateLimit-Reset")
                    if reset_time:
                        wait_time = int(reset_time) - int(time.time()) + 60
                        if wait_time > 0:
                            print(f"Rate limit hit. Waiting {wait_time} seconds...")
                            time.sleep(wait_time)
                            continue
                    else:
                        print("Rate limit hit. Waiting 60 seconds...")
                        time.sleep(60)
                        continue

                response.raise_for_status()
                return response.json()

            except requests.RequestException as e:
                if attempt < max_retries - 1:
                    print(f"Error fetching commit details for {commit_sha}: {e}")
                    print(
                        f"Retrying in 5 seconds... (attempt {attempt + 1}/{max_retries})"
                    )
                    time.sleep(5)
                else:
                    print(f"Error fetching commit details for {commit_sha}: {e}")
                    return None

        return None

    def save_detailed_output(self, commits: List[Dict], analysis: str):
        """Save detailed output files for each commit"""
        try:
            print("  Creating output directory...")
            # Create output directory
            os.makedirs(OUTPUT_DIR, exist_ok=True)

            # Create timestamp for this run
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            run_dir = os.path.join(OUTPUT_DIR, f"run_{timestamp}")
            os.makedirs(run_dir, exist_ok=True)
            print(f"  Output directory: {run_dir}")

            # Save the AI analysis
            analysis_file = os.path.join(run_dir, "ai_analysis.txt")
            with open(analysis_file, "w") as f:
                f.write("VESC Firmware Update Analysis\n")
                f.write("=" * 50 + "\n\n")
                f.write(analysis)

            print("  AI analysis saved")

            print(f"  Processing {len(commits)} commits...")
            # Save detailed information for each commit
            for i, commit in enumerate(commits):
                commit_sha = commit["sha"][:8]
                print(f"    Processing commit {i+1}/{len(commits)}: {commit_sha}")
                commit_dir = os.path.join(run_dir, f"commit_{i+1:02d}_{commit_sha}")
                os.makedirs(commit_dir, exist_ok=True)

                # Get detailed commit information
                commit_details = self.get_commit_details(commit["sha"])

                # Save commit metadata
                metadata = {
                    "sha": commit["sha"],
                    "sha_short": commit_sha,
                    "message": commit["commit"]["message"],
                    "author": commit["commit"]["author"]["name"],
                    "author_email": commit["commit"]["author"]["email"],
                    "date": commit["commit"]["author"]["date"],
                    "committer": commit["commit"]["committer"]["name"],
                    "committer_email": commit["commit"]["committer"]["email"],
                    "commit_date": commit["commit"]["committer"]["date"],
                    "url": commit["html_url"],
                    "parents": [p["sha"] for p in commit["parents"]],
                }

                with open(os.path.join(commit_dir, "metadata.json"), "w") as f:
                    json.dump(metadata, f, indent=2)

                # Save commit message
                with open(os.path.join(commit_dir, "message.txt"), "w") as f:
                    f.write(commit["commit"]["message"])

                # Save detailed file changes if available
                if commit_details and "files" in commit_details:
                    files_info = []
                    for file_info in commit_details["files"]:
                        file_data = {
                            "filename": file_info["filename"],
                            "status": file_info["status"],
                            "additions": file_info.get("additions", 0),
                            "deletions": file_info.get("deletions", 0),
                            "changes": file_info.get("changes", 0),
                            "blob_url": file_info.get("blob_url"),
                            "raw_url": file_info.get("raw_url"),
                            "contents_url": file_info.get("contents_url"),
                        }

                        # Include patch if available (for detailed analysis)
                        if "patch" in file_info:
                            file_data["patch"] = file_info["patch"]

                        files_info.append(file_data)

                    with open(os.path.join(commit_dir, "files.json"), "w") as f:
                        json.dump(files_info, f, indent=2)

                    # Save individual file patches
                    patches_dir = os.path.join(commit_dir, "patches")
                    os.makedirs(patches_dir, exist_ok=True)

                    patch_count = 0
                    for file_info in commit_details["files"]:
                        if "patch" in file_info:
                            filename = file_info["filename"].replace("/", "_")
                            patch_file = os.path.join(patches_dir, f"{filename}.patch")
                            with open(patch_file, "w") as f:
                                f.write(f"File: {file_info['filename']}\n")
                                f.write(f"Status: {file_info['status']}\n")
                                f.write(f"Additions: {file_info.get('additions', 0)}\n")
                                f.write(f"Deletions: {file_info.get('deletions', 0)}\n")
                                f.write("-" * 50 + "\n")
                                f.write(file_info["patch"])
                            patch_count += 1

                    if patch_count > 0:
                        print(f"      Saved {patch_count} patch files")

            print("  Creating summary file...")
            # Create a summary file
            summary_file = os.path.join(run_dir, "summary.txt")
            with open(summary_file, "w") as f:
                f.write(f"VESC Update Check Summary\n")
                f.write(f"Generated: {datetime.now().isoformat()}\n")
                f.write(f"Repository: {VESC_REPO}\n")
                f.write(f"Commits analyzed: {len(commits)}\n")
                f.write(f"Output directory: {run_dir}\n\n")

                f.write("Commits:\n")
                for i, commit in enumerate(commits):
                    message = commit["commit"]["message"].split("\n")[0]
                    f.write(f"{i+1:2d}. {commit['sha'][:8]} - {message}\n")

            print(f"  Summary file created")
            print(f"\nDetailed output saved to: {run_dir}")

        except Exception as e:
            print(f"Warning: Could not save detailed output: {e}")

    def analyze_commits_with_openai(self, commits: List[Dict]) -> str:
        """Use OpenAI to analyze commits and determine if SDK updates are needed"""
        if not commits:
            return "No new commits to analyze."

        # Prepare commit information for analysis
        print("  Fetching detailed commit information...")
        commit_info = []
        for i, commit in enumerate(commits):
            print(f"    Processing commit {i+1}/{len(commits)}: {commit['sha'][:8]}")
            commit_details = self.get_commit_details(commit["sha"])
            if commit_details:
                info = {
                    "sha": commit["sha"][:8],
                    "message": commit["commit"]["message"],
                    "author": commit["commit"]["author"]["name"],
                    "date": commit["commit"]["author"]["date"],
                    "files_changed": [
                        f["filename"] for f in commit_details.get("files", [])
                    ],
                    "additions": sum(
                        f.get("additions", 0) for f in commit_details.get("files", [])
                    ),
                    "deletions": sum(
                        f.get("deletions", 0) for f in commit_details.get("files", [])
                    ),
                }
                commit_info.append(info)
                print(f"      Found {len(info['files_changed'])} changed files")
            else:
                print(
                    f"      Warning: Could not fetch details for commit {commit['sha'][:8]}"
                )

        print(f"  Successfully processed {len(commit_info)} commits")
        print("  Sending to OpenAI for analysis...")

        # Create prompt for OpenAI
        prompt = f"""
You are an expert software developer analyzing VESC firmware commits to determine if they affect a VESC CAN SDK.

The VESC CAN SDK is a C library for communicating with VESC motor controllers over CAN bus. It includes:
- CAN packet handling and fragmentation
- Motor control commands (duty, current, RPM, etc.)
- Status and configuration commands
- Response parsing for various data types

Recent commits from the VESC firmware repository (for each commit: sha, message, author, date, files_changed, additions, deletions):

{json.dumps(commit_info, indent=2)}

IMPORTANT FILTERING CRITERIA:
- IGNORE commits that do not touch CAN-related files (comm.c, comm.h, can.c, can.h, packet.c, packet.h, etc.)
- IGNORE commits that only modify documentation, build scripts, or non-communication code
- IGNORE commits that are unlikely to affect the SDK (UI changes, hardware-specific code, etc.)
- ONLY consider commits that modify communication protocol, command structures, or response formats

Please analyze these commits and determine if any changes require updates to the VESC CAN SDK. Consider:

1. Changes to CAN communication protocol
2. New or modified commands
3. Changes to packet structures
4. New response data formats
5. Changes to command IDs or packet types
6. Protocol version changes

If changes are needed, provide a clear, actionable prompt that can be given to a coding assistant to implement the necessary updates. The prompt should be specific and include:
- What needs to be changed
- Where the changes should be made
- Specific implementation details

If no changes are needed or if commits should be ignored based on the filtering criteria, clearly state that no SDK updates are required and briefly explain why the commits were ignored.

Focus ONLY on changes that affect the communication protocol, not internal firmware changes.
"""

        try:
            print("  Connecting to OpenAI...")
            client = openai.OpenAI(api_key=self.api_key)
            print("  Waiting for AI analysis (this may take a moment)...")
            response = client.chat.completions.create(
                model="gpt-4",
                messages=[
                    {
                        "role": "system",
                        "content": "You are an expert software developer specializing in embedded systems and communication protocols.",
                    },
                    {"role": "user", "content": prompt},
                ],
                max_tokens=2000,
                temperature=0.1,
            )

            print("  Analysis complete!")
            return response.choices[0].message.content

        except Exception as e:
            return f"Error analyzing commits with OpenAI: {e}"

    def run(self):
        """Main execution function"""
        print("VESC Firmware Update Checker")
        print("=" * 40)
        print(f"Repository: {VESC_REPO}")
        print(f"Last checked commit: {self.last_checked_commit or 'None (first run)'}")
        print()

        # Get recent commits
        commits = self.get_recent_commits()

        if not commits:
            print("No new commits found since last check.")
            return

        print(f"Found {len(commits)} new commits to analyze:")
        for commit in commits:
            message = commit["commit"]["message"].split("\n")[0]
            print(f"  {commit['sha'][:8]} - {message}")
        print()

        # Analyze commits
        print("Analyzing commits with OpenAI...")
        print(f"  Sending {len(commits)} commits for analysis...")
        analysis = self.analyze_commits_with_openai(commits)

        print()
        print("=" * 40)
        print("ANALYSIS RESULTS")
        print("=" * 40)
        print(analysis)
        print("=" * 40)

        # Save detailed output files
        print("\nGenerating detailed output files...")
        self.save_detailed_output(commits, analysis)

        # Save the latest commit as checked
        if commits:
            self.save_last_checked_commit(commits[0]["sha"])
            print()
            print(f"Updated last checked commit to: {commits[0]['sha'][:8]}")


def main():
    parser = argparse.ArgumentParser(
        description="Check VESC firmware updates for SDK compatibility"
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Force check all recent commits (ignore last checked)",
    )
    args = parser.parse_args()

    checker = VESCUpdateChecker()

    if args.force:
        # Remove the last checked commit file to force a full check
        if os.path.exists(COMMIT_CHECK_FILE):
            os.remove(COMMIT_CHECK_FILE)
            checker.last_checked_commit = None
            print("Force mode: Will check all recent commits")

    checker.run()


if __name__ == "__main__":
    main()
