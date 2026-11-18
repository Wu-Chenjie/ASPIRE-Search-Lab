"""Team intent records shared by peer replication and lease accounting."""


def intent_records(peer):
    return [peer[key] for key in ('intent', 'pending_intent') if peer.get(key)]+list(peer.get('retiring_intents', []))
