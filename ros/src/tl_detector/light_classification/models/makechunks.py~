# chunked file reading
from __future__ import division
import os

def get_chunks(file_size):
    chunk_start = 0
    chunk_size = 0x2710000  # 131072 bytes, default max ssl buffer size
    while chunk_start + chunk_size < file_size:
        yield(chunk_start, chunk_size)
        chunk_start += chunk_size

    final_chunk_size = file_size - chunk_start
    yield(chunk_start, final_chunk_size)

def read_file_chunked(file_path, directory):
    if not os.path.exists(directory):
        os.mkdir(directory)
    else:
        for fname in os.listdir(directory):
            os.remove(os.path.join(directory, fname))
    with open(file_path, 'rb') as file_:
        file_size = os.path.getsize(file_path)

        print('File size: {}'.format(file_size))

        progress = 0

        for chunk_start, chunk_size in get_chunks(file_size):
            file_chunk = file_.read(chunk_size)
            ofilename = os.path.join(directory, ('chunk%04d'%(chunk_start)))
            outfile = open(ofilename, 'wb')
            outfile.write(file_chunk)
            outfile.close()
            progress += len(file_chunk)
            print('{0} of {1} bytes read ({2}%)'.format(
                progress, file_size, int(progress / file_size * 100))
            )

if __name__ == '__main__':
    read_file_chunked('frozen_inference_graph.pb', 'chunks')
