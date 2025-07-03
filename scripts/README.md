# VESC CAN SDK Scripts

This directory contains utility scripts for the VESC CAN SDK.

## VESC Firmware Update Checker

The `check_vesc_updates.py` script monitors the VESC firmware repository for changes that might affect the SDK.

### Features

- Monitors the [VESC firmware repository](https://github.com/vedderb/bldc) for new commits
- Uses OpenAI to analyze commits and determine if SDK updates are needed
- Tracks the last checked commit to avoid re-analyzing the same commits
- Provides actionable prompts for implementing necessary changes
- Supports force mode to re-check all recent commits

### Setup

1. **Install dependencies:**
   ```bash
   cd scripts
   python -m venv venv
   source ./venv/bin/activate
   pip install -r scripts/requirements.txt
   ```

2. **Set up API keys:**
   Create a `.env` file in the project root with your API keys:
   ```
   OPENAI_API_KEY=your_openai_api_key_here
   GITHUB_TOKEN=your_github_token_here
   ```
   
   Or set them as environment variables:
   ```bash
   export OPENAI_API_KEY=your_openai_api_key_here
   export GITHUB_TOKEN=your_github_token_here
   ```
   
   **GitHub Token (Optional but Recommended):**
   - Without a token: 60 requests/hour limit
   - With a token: 5000 requests/hour limit
   - Create a token at: https://github.com/settings/tokens
   - No special permissions needed (public repo access)

### Usage

**Basic usage:**
```bash
python check_vesc_updates.py
```

**Force check all recent commits:**
```bash
python check_vesc_updates.py --force
```

### Output

The script will:
1. Check for new commits since the last run
2. Analyze commits using OpenAI to determine if SDK updates are needed
3. Provide a detailed analysis with actionable prompts if changes are required
4. Save detailed output files for each commit in the `output/` directory
5. Update the last checked commit for future runs

### Output Directory Structure

When the script runs, it creates a timestamped directory in `output/` with the following structure:

```
output/
└── run_20241201_143022/
    ├── ai_analysis.txt          # AI analysis and recommendations
    ├── summary.txt              # Summary of the analysis run
    └── commit_01_a1b2c3d4/      # Directory for each commit
        ├── metadata.json        # Commit metadata (SHA, author, date, etc.)
        ├── message.txt          # Full commit message
        ├── files.json           # List of changed files with details
        └── patches/             # Individual patch files for each changed file
            ├── file1.c.patch
            └── file2.h.patch
```

### Output Files Explained

- **`ai_analysis.txt`**: Contains the AI's analysis and any actionable prompts for SDK updates
- **`summary.txt`**: Overview of the analysis run with commit list
- **`metadata.json`**: Complete commit information (SHA, author, dates, URLs)
- **`message.txt`**: Full commit message for context
- **`files.json`**: Detailed information about each changed file
- **`patches/*.patch`**: Individual diff files showing exactly what changed in each file

### Example Output

```
VESC Firmware Update Checker
========================================
Repository: vedderb/bldc
Last checked commit: None (first run)

Found 5 new commits to analyze:
  60cdd9bb - Added support for HW_NO_ABS_MAX_CALC flag
  3879278f - Added more settings, use milliohm in conf-detect-lambda-enc for consistency
  edd94d54 - Fixed typo in doc
  8bf72aa2 - Added observer type and mtpa mode parameters, not app default on maxims
  e891bb2e - Merge commit '944b0ee085f6c695c82d072b009494c9a33d0eae'

Analyzing commits with OpenAI...
  Sending 5 commits for analysis...
  Fetching detailed commit information...
    Processing commit 1/5: 60cdd9bb
      Found 1 changed files
    Processing commit 2/5: 3879278f
      Found 2 changed files
    Processing commit 3/5: edd94d54
      Found 1 changed files
    Processing commit 4/5: 8bf72aa2
      Found 5 changed files
    Processing commit 5/5: e891bb2e
      Found 5 changed files
  Successfully processed 5 commits
  Sending to OpenAI for analysis...
  Connecting to OpenAI...
  Waiting for AI analysis (this may take a moment)...
  Analysis complete!

========================================
ANALYSIS RESULTS
========================================
Based on the provided commit history and the filtering criteria, none of the commits require updates to the VESC CAN SDK. Here's why:

1. Commit "60cdd9bb": This commit only modifies the "conf_general.c" file, which is not related to CAN communication. Therefore, it does not affect the VESC CAN SDK.

2. Commit "3879278f": This commit modifies the "lispBM/README.md" and "lispBM/lispif_vesc_extensions.c" files. The README file is a documentation file, and the "lispif_vesc_extensions.c" file is not related to CAN communication. Therefore, this commit does not affect the VESC CAN SDK.

3. Commit "edd94d54": This commit only fixes a typo in the "lispBM/README.md" file, which is a documentation file. Therefore, it does not affect the VESC CAN SDK.

4. Commit "8bf72aa2": This commit modifies several files, but none of them are related to CAN communication. Therefore, it does not affect the VESC CAN SDK.

5. Commit "e891bb2e": This commit modifies several files in the "lispBM" directory, but none of them are related to CAN communication. Therefore, it does not affect the VESC CAN SDK.

In conclusion, none of the provided commits modify the CAN communication protocol, command structures, packet structures, response data formats, command IDs, packet types, or protocol version. Therefore, no updates are required for the VESC CAN SDK.
========================================

Generating detailed output files...
  Creating output directory...
  Output directory: output/run_20250703_125506
  AI analysis saved
  Processing 5 commits...
    Processing commit 1/5: 60cdd9bb
      Saved 1 patch files
    Processing commit 2/5: 3879278f
      Saved 2 patch files
    Processing commit 3/5: edd94d54
      Saved 1 patch files
    Processing commit 4/5: 8bf72aa2
      Saved 5 patch files
    Processing commit 5/5: e891bb2e
      Saved 5 patch files
  Creating summary file...
  Summary file created

Detailed output saved to: output/run_20250703_125506

Updated last checked commit to: 60cdd9bb
```

### Files

- `check_vesc_updates.py` - Main script
- `requirements.txt` - Python dependencies
- `last_checked_commit.txt` - Tracks the last checked commit (created automatically)

### Notes

- The script uses the GitHub API to fetch commit information
- OpenAI GPT-4 is used for intelligent analysis of commits
- Only commits that affect the communication protocol are flagged for SDK updates
- The script is designed to be run regularly to keep the SDK up to date with firmware changes
- Uses OpenAI API v1.0.0+ (new client format) 