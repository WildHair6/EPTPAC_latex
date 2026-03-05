import re

bib = open('sample.bib', encoding='utf-8').read()
keys = ['sanchez-torres_class_2018', 'xiao_predefined-time_2025', 'das_predefined-time_2024', 'xu_distributed_2024', 'meng_predefined-time_2025', 'song_time-varying_2017', 'zhou_finite-time_2020', 'wang_prescribed-time_2019', 'cao_practical_2022', 'xiao_scaling_2025', 'ye_predefined-time_2022', 'zhou_fixed-time_2022', 'ding_strong_2024', 'zhou_prescribed_2024']
out = []
for entry in bib.split('@')[1:]:
    key = entry.split(',')[0].strip().split('{')[-1]
    if key in keys:
        author = re.search(r'author\s*=\s*{([^}]+)}', entry, re.I | re.DOTALL)
        title = re.search(r'title\s*=\s*{([^}]+)}', entry, re.I | re.DOTALL)
        journal = re.search(r'journal\s*=\s*{([^}]+)}', entry, re.I | re.DOTALL)
        year = re.search(r'year\s*=\s*{?(\d{4})}?', entry, re.I)
        vol = re.search(r'volume\s*=\s*{?(\d+)}?', entry, re.I)
        num = re.search(r'number\s*=\s*{?([^}]+)}?', entry, re.I)
        page = re.search(r'pages\s*=\s*{?([\d\-]+)}?', entry, re.I)
        
        a = author.group(1).replace('\n', ' ').replace('\t', '') if author else 'Unknown'
        t = title.group(1).replace('\n', ' ').replace('{', '').replace('}', '') if title else 'Unknown'
        j = journal.group(1).replace('\n', ' ') if journal else 'Unknown'
        y = year.group(1) if year else ''
        v = vol.group(1) if vol else ''
        n = num.group(1) if num else ''
        p = page.group(1) if page else ''
        
        alist = [x.strip() for x in a.split(' and ')]
        names = []
        for x in alist:
            parts = x.split(',')
            if len(parts) == 2:
                names.append(parts[0].strip() + ' ' + ''.join([c[0] for c in parts[1].split() if c]))
            else:
                names.append(x)
        a_str = ', '.join(names[:3]) + (', et al' if len(names)>3 else '')
        
        s = f'{a_str}. {t} [J]. {j}, {y}, {v}({n}): {p}'
        out.append(s)

with open('dump.txt', 'w', encoding='utf-8') as f:
    f.write('\n'.join(out))
