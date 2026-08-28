import glob, os

for name in ['ABE', 'BEN', 'TARGET', 'MOTHERSHIP', 'SHORESIDE']:
    dirs = sorted(glob.glob(f'/home/yoei/moos-ivp/ivp/missions/m_shield_demo/*{name}*12_03*'))
    if not dirs:
        continue
    d = dirs[-1]
    alogs = glob.glob(d + '/*.alog')
    if not alogs:
        continue
    alog = alogs[0]
    print(f'=== [{name}] ===')
    with open(alog, 'r', errors='ignore') as fp:
        lines = fp.readlines()
    
    # Check BHV_ERROR, BHV_WARNING, ALLSTOP, IVPHELM_STATE, MODE, NAV_X, NAV_Y, NAV_SPEED, NAV_HEADING
    interesting = ['BHV_ERROR', 'BHV_WARNING', 'IVPHELM_STATE', 'MODE', 'NAV_X', 'NAV_Y', 'NAV_SPEED', 'NAV_HEADING', 'SHIELD_STATE', 'TARGET_COORDINATOR_STATUS']
    latest = {}
    errors = []
    for l in lines:
        parts = l.strip().split()
        if len(parts) >= 4:
            var = parts[1]
            if var in interesting:
                latest[var] = parts[0] + ': ' + ' '.join(parts[2:])
            if 'ERROR' in var or 'WARNING' in var or 'ALLSTOP' in var or 'ERROR' in l:
                errors.append(l.strip())
    
    for k, v in latest.items():
        print(f'  {k}: {v}')
    if errors:
        print('  Recent Warnings/Errors:')
        for e in errors[-4:]:
            print(f'    {e}')
    print()
