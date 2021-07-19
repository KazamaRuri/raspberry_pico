def ByteToHex( bins ):
    return ''.join( [ "%02X" % x for x in bins ] ).strip()