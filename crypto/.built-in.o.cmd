cmd_crypto/built-in.o :=  arm-eabi-ld -EL    -r -o crypto/built-in.o crypto/crypto.o crypto/crypto_algapi.o crypto/aead.o crypto/crypto_blkcipher.o crypto/chainiv.o crypto/eseqiv.o crypto/crypto_hash.o crypto/cryptomgr.o crypto/hmac.o crypto/crypto_null.o crypto/md5.o crypto/sha1_generic.o crypto/ecb.o crypto/cbc.o crypto/des_generic.o crypto/twofish.o crypto/twofish_common.o crypto/aes_generic.o crypto/arc4.o crypto/deflate.o crypto/authenc.o crypto/rng.o crypto/krng.o 
