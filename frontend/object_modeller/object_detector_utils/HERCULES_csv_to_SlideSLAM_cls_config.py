import csv
import argparse
import os
import re
import nltk
from nltk.corpus import words

# Download the word database (only needs to be done once)
nltk.download('words', quiet=True)
ENGLISH_WORDS = set(word.lower() for word in words.words())

def get_longest_english_subword(chunk):
    """
    Generates every possible subword and returns the longest one 
    present in the dictionary, prioritizing words > 1 char.
    """
    n = len(chunk)
    longest_word = ""
    
    for i in range(n):
        for j in range(i + 1, n + 1):
            subword = chunk[i:j].lower()
            if subword in ENGLISH_WORDS:
                # Prioritize length, but we will filter 1-char words later
                if len(subword) > len(longest_word):
                    longest_word = subword
                    
    return longest_word if longest_word else None

def extract_categories(label):
    """
    Finds the best English word in the label. 
    Strictly ignores single-character results to avoid 'C', 'N', 'S', etc.
    """
    chunks = re.findall(r'[a-zA-Z]+', label)
    best_word = ""
    
    for chunk in chunks:
        found_word = get_longest_english_subword(chunk)
        if found_word and len(found_word) > len(best_word):
            best_word = found_word
                
    # Only return if the word is robust (more than 1 character)
    return best_word if len(best_word) > 1 else None

def generate_hercules_cls_config(input_csv_path, output_yaml_path):
    output_lines = []
    added_categories = set()
    next_id = 1  # Counter for auto-incrementing ID - Starts at 1 cause 0 means no class

    if not os.path.exists(input_csv_path):
        print(f"Error: Input file not found at {input_csv_path}")
        return

    os.makedirs(os.path.dirname(output_yaml_path), exist_ok=True)

    with open(input_csv_path, mode='r', encoding='utf-8') as csvfile:
        reader = csv.DictReader(csvfile)
        
        for row in reader:
            full_label = row['Label'].strip()
            category = extract_categories(full_label)
            
            if category:
                category_key = category.capitalize()
                
                if category_key not in added_categories:
                    added_categories.add(category_key)
                    
                    # Normalize RGB
                    r = round(float(row['R']) / 255.0, 2)
                    g = round(float(row['G']) / 255.0, 2)
                    b = round(float(row['B']) / 255.0, 2)
                    
                    output_lines.append(f"{category_key}:")
                    output_lines.append(f"  id: {next_id}")
                    output_lines.append(f"  color: [{r}, {g}, {b}]")
                    output_lines.append(f"  length_cutoff: [0.0, 30.0]")
                    output_lines.append(f"  height_cutoff: [0.0, 30.0]")
                    output_lines.append(f"  mesh_model_path: null")
                    output_lines.append(f"  mesh_model_scale: null")
                    output_lines.append(f"  class_assignment_thresh: 3.0")
                    output_lines.append("") 
                    
                    # Increment ID only after a unique category is successfully added
                    next_id += 1

    with open(output_yaml_path, 'w', encoding='utf-8') as yamlfile:
        yamlfile.write("\n".join(output_lines))
    
    print(f"Successfully wrote config to {output_yaml_path}")
    print(f"Unique English categories extracted: {', '.join(sorted(added_categories))}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate Hercules config with auto-incrementing IDs.")
    parser.add_argument("--dataset_num", type=str, required=True, help="Dataset version (e.g., V2.3.C)")
    
    args = parser.parse_args()
    ds = args.dataset_num

    input_path = f'/home/dbutterfield3/data/Hercules_datasets/{ds}/data/label_color_map.csv'
    output_path = f'/home/dbutterfield3/slideslam_original_ws/src/SLIDE_SLAM/frontend/object_modeller/config/Hercules/{ds}/hercules_cls_config.yaml'

    generate_hercules_cls_config(input_path, output_path)