import sys
import key_handling

if __name__ == '__main__':
    args = len(sys.argv) - 1

    if args < 1:
        print('Please specify key, e.g. a10 or 6989586621679009802')

    key = sys.argv[1]

    print('Provided key:', key)
    print('   GTSAM key: ', end='')

    if key[0].isalpha():
        print(key_handling.join_pg_key(key[0], int(key[1:])))

    else:
        print(*key_handling.split_pg_key(key), sep='')
