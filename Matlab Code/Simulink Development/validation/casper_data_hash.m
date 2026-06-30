function h = casper_data_hash(x)
%CASPER_DATA_HASH MD5 hash of a MATLAB value's byte-stream representation.
%
% Synopsis:
%   h = casper_data_hash(x)
%
% Inputs:
%   x : any MATLAB value (struct, array, table, ...)
%
% Outputs:
%   h : char row vector, 32 hex chars (lowercase) MD5 digest
%
% Implementation: getByteStreamFromArray(x) -> MD5 via Java MessageDigest.
% This is deterministic across MATLAB sessions for byte-identical content.

    bytes = getByteStreamFromArray(x);
    md = java.security.MessageDigest.getInstance('MD5');
    md.update(bytes);
    digest = typecast(md.digest(), 'uint8');
    h = lower(reshape(dec2hex(digest, 2).', 1, []));
end
