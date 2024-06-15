import os
import shutil

def copy_markdown_files(src_dir, dest_dir):
    for root, dirs, files in os.walk(src_dir):
        for file in files:
            if file.endswith(".md"):
                src_path = os.path.join(root, file)
                rel_path = os.path.relpath(src_path, src_dir)
                dest_path = os.path.join(dest_dir, rel_path)
                os.makedirs(os.path.dirname(dest_path), exist_ok=True)
                shutil.copy2(src_path, dest_path)

# Usage
repo_root = "../../"
docs_dir = os.path.join(repo_root, "docs")
copy_markdown_files(repo_root, docs_dir)
