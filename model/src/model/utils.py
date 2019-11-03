import re

def bundle_msg_to_dict(bundle):
    # Convert bundle messages to dictionaries
    audio_p1 = msg_to_dict(bundle.audio_features_P1)
    audio_p2 = msg_to_dict(bundle.audio_features_P2)
    audio_p3 = msg_to_dict(bundle.audio_features_P3)
    # Add pid
    audio_p1 = add_pid(audio_p1, 'P1')
    audio_p2 = add_pid(audio_p2, 'P2')
    audio_p3 = add_pid(audio_p3, 'P3')
    out = {}
    out.update(audio_p1)
    out.update(audio_p2)
    out.update(audio_p3)
    return out

def add_pid(msg, pid):
    out = {}
    for key in msg:
        out[key + '_' + pid] = msg[key]
    return out

def msg_to_dict(msg):
    dictionary = {}
    message_fields = zip(msg.__slots__, msg._slot_types)
    for field_name, field_type in message_fields:
        # Skip the header, as it is not depth 1
        if field_name == 'header':
            continue
        field_value = getattr(msg, field_name)
        dictionary[field_name] = str(field_value)

    return dictionary

def create_pairwise_rows(row):
    rows = []
    done = []
    for actor in [1,2,3]:
        for receiver in [1,2,3]:
            for bystander in [1,2,3]:
                if len(set((actor, receiver, bystander))) != len((actor, receiver, bystander)):
                    continue
                new_row = substitute_ids(row, actor, receiver, bystander)
                new_row['actor'] = actor
                new_row['receiver'] = receiver
                new_row['bystander'] = bystander
                rows.append(new_row)
    return rows

def substitute_ids(row, actor, receiver, bystander):
    new_row = {}
    for key in row:
        # Substitute actor id with _actor
        new_key, n = re.subn(r'_P{}$'.format(actor), '_actor', key)
        if not n:
            # Substitute receiver id with _receiver
            new_key, n = re.subn(r'_P{}$'.format(receiver), '_receiver', key)
            if not n:
                # Substitute bystander id with _bystander
                new_key, n = re.subn(r'_P{}$'.format(bystander), '_bystander', key)
        new_row[new_key] = row[key]
    return new_row
