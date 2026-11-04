function h = md5_file(file)
%MD5_FILE  Hash MD5 di un file (tramite Java); '' se non disponibile.
h = '';
try
    fid = fopen(file, 'r');
    b = fread(fid, inf, '*uint8');
    fclose(fid);
    md = java.security.MessageDigest.getInstance('MD5');
    md.update(typecast(b, 'int8'));
    d = typecast(md.digest(), 'uint8');
    h = lower(reshape(dec2hex(d, 2)', 1, []));
catch
end
end
