import sys
import key_handling

if __name__ == '__main__':
    args = len(sys.argv) - 1

    if args < 1:
        print('Please specify key, e.g. a10')

    key = sys.argv[1]

    print('Provided key:', key)
    print('GTSAM key:')
    print(key_handling.join_pg_key(key[0], int(key[1:])))
