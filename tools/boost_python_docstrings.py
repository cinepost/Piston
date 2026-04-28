import re
import sys
import argparse
from pathlib import Path

def get_cxx_files(directory):
    path = Path(directory)
    # Define common C++ header extensions
    extensions = ("*.h", "*.hpp")
    
    file_names = list()

    for ext in extensions:
        for cxx_file_name in path.rglob(ext):
            file_names.append(cxx_file_name)

    return file_names

def parse_doc_string(istr):
    pattern = re.compile(r'@(\w+)\s+(.*)')
    docstring = list()
    for line in map(lambda s : s.strip(), istr):
        if line == '/**':
            continue
        if line == '*/':
            return docstring
        line = line.lstrip('* ')
        match = pattern.match(line)
        if match:
            docstring.append((match.group(1), match.group(2)))

def extract(istr, docstrings):
    pattern = re.compile(r'^//\s*DocString:\s*(\w+)$')
    for line in map(lambda s : s.strip(), istr):
        match = pattern.match(line)
        if match:
            token = match.group(1)
            docstrings[token] = parse_doc_string(istr)

def format_doc_string(docstring):
    return '\n'.join('{}: {}'.format(k, v) for (k, v) in docstring)

def escape(string):
    return string.replace('\n', r'\n')

def substitute(istr, ostr, docstrings):
    pattern = re.compile(r'@DocString\((\w+)\)')
    for line in map(lambda s : s.rstrip(), istr):
        for match in pattern.finditer(line):
            token = match.group(1)
            docstring = format_doc_string(docstrings[token])
            line = line.replace(match.group(0), escape(docstring))
        print(line, file=ostr)

def main():
    parser = argparse.ArgumentParser(description='Configure C++ source from template.')
    parser.add_argument('--template', required=True, help='Path to .cpp template')
    parser.add_argument('--output', required=True, help='Path for generated .cpp file')
    parser.add_argument('--srcdir', required=True, help='Root path for c++ header files')
    parser.add_argument('--version', default='1.0.0')

    args = parser.parse_args()

    try:
        docstrings = dict()
        cxx_file_names = get_cxx_files(args.srcdir)
            
        # search for cpp/h files and extract doc strings
        for cxx_file_name in cxx_file_names:
            with open(cxx_file_name, 'r') as in_file:
                extract(in_file, docstrings)

        # substitute docstrings
        with open(args.template, 'r') as in_file:
            with open(args.output, 'w') as out_file:
                substitute(in_file, out_file, docstrings)

    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()