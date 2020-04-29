from __future__ import absolute_import, print_function

import subprocess, sys, os, re, tempfile, zipfile, gzip, io, shutil, string, random, itertools, pickle, json, yaml, gc
from datetime import datetime
from time import time
from fnmatch import fnmatch
from glob import glob
from tqdm import tqdm
from copy import copy, deepcopy
from collections import OrderedDict, defaultdict, Counter
import q
qq = q
import warnings
warnings.filterwarnings('ignore')

version = sys.version_info
if version[0] < 3:
    from StringIO import StringIO
else:
    from io import StringIO

def from_args():
    import argparse
    parser = argparse.ArgumentParser(description='Model arguments')
    parser.add_argument('kwargs', nargs='*', help='Extra arguments that goes into the config')

    args = parser.parse_args()

    kwargs = dict()
    for kv in args.kwargs:
        splits = kv.split('=')
        if len(splits) == 1:
            v = True
        else:
            v = splits[1]
            try:
                v = eval(v)
            except (SyntaxError, NameError):
                pass
        kwargs[splits[0]] = v

    return Namespace(**kwargs)

def lrange(*args, **kwargs):
    return list(range(*args, **kwargs))

def parse_dot(d):
    ks = [(k, v) for k, v in d.items() if '.' in k]
    for k, v in ks:
        del d[k]
        curr = d
        *fronts, back = k.split('.')
        for k_ in fronts:
            curr = curr.setdefault(k_, {})
        curr[back] = v
    return d

def load_json(path):
    with open(path, 'r+') as f:
        return json.load(f)

def save_json(path, dict_):
    with open(path, 'w+') as f:
        json.dump(dict_, f, indent=4, sort_keys=True)

def format_json(dict_):
    return json.dumps(dict_, indent=4, sort_keys=True)

def format_yaml(dict_):
    dict_ = recurse(dict_, lambda x: x._ if type(x) is Path else dict(x) if type(x) is dict else x)
    return yaml.dump(dict_)

def load_text(path, encoding='utf-8'):
    with open(path, 'r', encoding=encoding) as f:
        return f.read()

def save_text(path, string):
    with open(path, 'w') as f:
        f.write(string)

def load_pickle(path):
    with open(path, 'rb') as f:
        return pickle.load(f)

def save_pickle(path, obj):
    with open(path, 'wb') as f:
        pickle.dump(obj, f)

def wget(link, output_dir):
    cmd = 'wget %s -P %s' % (path, output_dir)
    shell(cmd)
    output_path = Path(output_dir) / os.path.basename(link)
    if not output_path.exists(): raise RuntimeError('Failed to run %s' % cmd)
    return output_path

def extract(input_path, output_path=None):
    if input_path[-3:] == '.gz':
        if not output_path:
            output_path = input_path[:-3]
        with gzip.open(input_path, 'rb') as f_in:
            with open(output_path, 'wb') as f_out:
                f_out.write(f_in.read())
    else:
        raise RuntimeError('Don\'t know file extension for ' + input_path)

def rand_string(length):
    import string
    letters = string.ascii_lowercase
    return ''.join(random.choice(letters) for i in range(length))

def shell(cmd, wait=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE):
    stdout = stdout or subprocess.DEVNULL
    stderr = stderr or subprocess.DEVNULL
    if type(cmd) != str:
        cmd = ' '.join(cmd)
    process = subprocess.Popen(cmd, shell=True, stdout=stdout, stderr=stderr)
    if not wait:
        return process
    out, err = process.communicate()
    return out.decode().rstrip('\n') if out else None, err.decode().rstrip('\n') if err else None

def attributes(obj, print=True):
    import inspect
    attrs = inspect.getmembers(obj, lambda a: not inspect.isroutine(a))
    if print:
        from pprint import pprint
        pprint(attrs)
    return attrs

class Path(str):
    def __init__(self, path='.'):
        pass

    def __add__(self, subpath):
        return Path(str(self) + str(subpath))

    def __truediv__(self, subpath):
        return Path(os.path.join(str(self), str(subpath)))

    def __floordiv__(self, subpath):
        return (self / subpath)._

    def ls(self, show_hidden=True, dir_only=False, file_only=False):
        subpaths = [Path(self / subpath) for subpath in os.listdir(self) if show_hidden or not subpath.startswith('.')]
        isdirs = [os.path.isdir(subpath) for subpath in subpaths]
        subdirs = [subpath for subpath, isdir in zip(subpaths, isdirs) if isdir]
        files = [subpath for subpath, isdir in zip(subpaths, isdirs) if not isdir]
        if dir_only:
            return subdirs
        if file_only:
            return files
        return subdirs, files

    def lsdirs(self, show_hidden=True):
        return self.ls(show_hidden=show_hidden, dir_only=True)

    def lsfiles(self, show_hidden=True):
        return self.ls(show_hidden=show_hidden, file_only=True)

    def lslinks(self, show_hidden=True, exist=None):
        dirs, files = self.ls(show_hidden=show_hidden)
        return [x for x in dirs + files if x.islink() and (
            exist is None or not (exist ^ x.exists()))]

    def glob(self, glob_str):
        return [Path(p) for p in glob(self / glob_str, recursive=True)]

    def recurse(self, dir_fn=None, file_fn=None):
        if dir_fn is not None:
            dir_fn(self)
        dirs, files = self.ls()
        if file_fn is not None:
            list(map(file_fn, files))
        for dir in dirs:
            dir.recurse(dir_fn=dir_fn, file_fn=file_fn)

    def mk(self):
        os.makedirs(self, exist_ok=True)
        return self

    def rm(self):
        if self.isfile() or self.islink():
            os.remove(self)
        elif self.isdir():
            shutil.rmtree(self)
        return self

    def mv(self, dest):
        shutil.move(self, dest)

    def mv_from(self, src):
        shutil.move(src, self)

    def cp(self, dest):
        shutil.copy(self, dest)

    def cp_from(self, src):
        shutil.copy(src, self)

    def link(self, target, force=False):
        if self.lexists():
            if not force:
                return
            else:
                self.rm()
        os.symlink(target, self)

    def exists(self):
        return os.path.exists(self)

    def lexists(self):
        return os.path.lexists(self)

    def isfile(self):
        return os.path.isfile(self)

    def isdir(self):
        return os.path.isdir(self)

    def islink(self):
        return os.path.islink(self)

    def rel(self, start=None):
        return Path(os.path.relpath(self, start=start))

    def clone(self):
        name = self._name
        match = re.search('__([0-9]+)$', name)
        if match is None:
            base = self + '__'
            i = 1
        else:
            initial = match.group(1)
            base = self[:-len(initial)]
            i = int(initial) + 1
        while True:
            path = Path(base + str(i))
            if not path.exists():
                return path
            i += 1


    @property
    def _(self):
        return str(self)

    @property
    def _real(self):
        return Path(os.path.realpath(os.path.expanduser(self)))

    @property
    def _up(self):
        path = os.path.dirname(self.rstrip('/'))
        if path is '':
            path = os.path.dirname(self._real.rstrip('/'))
        return Path(path)

    @property
    def _name(self):
        return os.path.basename(self)

    @property
    def _ext(self):
        frags = self._name.rsplit('.', 1)
        if len(frags) == 1:
            return ''
        return frags[1]

    extract = extract
    load_json = load_json
    save_json = save_json
    load_txt = load_text
    save_txt = save_text
    load_p = load_pickle
    save_p = save_pickle

    def save_bytes(self, bytes):
        with open(self, 'wb') as f:
            f.write(bytes)

    def load_csv(self, index_col=0, **kwargs):
        return pd.read_csv(self, index_col=index_col, **kwargs)

    def save_csv(self, df, float_format='%.5g', **kwargs):
        df.to_csv(self, float_format=float_format, **kwargs)

    def load_npy(self):
        return np.load(self, allow_pickle=True)

    def save_npy(self, obj):
        np.save(self, obj)

    def load_yaml(self):
        with open(self, 'r') as f:
            return yaml.safe_load(f)

    def save_yaml(self, obj):
        obj = recurse(obj, lambda x: x._ if type(x) is Path else dict(x) if type(x) is dict else x)
        with open(self, 'w') as f:
            yaml.dump(obj, f, default_flow_style=False, allow_unicode=True)

    def load_pdf(self):
        """
        return: PdfReader object.
        Can use index and slice obj.pages for the pages, then call Path.save_pdf to save
        """
        from pdfrw import PdfReader
        return PdfReader(self)

    def save_pdf(self, pages):
        from pdfrw import PdfWriter
        writer = PdfWriter()
        writer.addpages(pages)
        writer.write(self)

    def load(self):
        return eval('self.load_%s' % self._ext)()

    def save(self, obj):
        return eval('self.save_%s' % self._ext)(obj)

    def wget(self, link):
        if self.isdir():
            return Path(wget(link, self))
        raise ValueError('Path %s needs to be a directory' % self)


class Namespace(dict):
    def __init__(self, *args, **kwargs):
        self.var(*args, **kwargs)

    def var(self, *args, **kwargs):
        kvs = dict()
        for a in args:
            if type(a) is str:
                kvs[a] = True
            else: # a is a dictionary
                kvs.update(a)
        kvs.update(kwargs)
        self.update(kvs)
        return self

    def unvar(self, *args):
        for a in args:
            self.pop(a)
        return self

    def setdefaults(self, *args, **kwargs):
        args = [a for a in args if a not in self]
        kwargs = {k: v for k, v in kwargs.items() if k not in self}
        return self.var(*args, **kwargs)

    def __getattr__(self, key):
        try:
            return self[key]
        except KeyError as e:
            self.__getattribute__(key)

    def __setattr__(self, key, value):
        self[key] = value

##### Functions for compute

using_ipython = True
try:
    _ = get_ipython().__class__.__name__
except NameError:
    using_ipython = False

try:
    import numpy as np
    import pandas as pd

    import scipy.stats
    import scipy as sp
    from scipy.stats import pearsonr as pearson, spearmanr as spearman, kendalltau

    if not using_ipython:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt_colors = plt.rcParams['axes.prop_cycle'].by_key()['color']

    def _sel(self, col, value):
        if type(value) == list:
            return self[self[col].isin(value)]
        return self[self[col] == value]
    pd.DataFrame.sel = _sel
except ImportError:
    pass
try:
    from sklearn.metrics import roc_auc_score as auroc, average_precision_score as auprc, roc_curve as roc, precision_recall_curve as prc, accuracy_score as accuracy
except ImportError:
    pass

def flatten(x):
    return [z for y in x for z in y]

def recurse(x, fn):
    T = type(x)
    if T in [dict, OrderedDict, dict]:
        return T((k, recurse(v, fn)) for k, v in x.items())
    elif T in [list, tuple]:
        return T(recurse(v, fn) for v in x)
    return fn(x)

def from_numpy(x):
    def helper(x):
        if type(x).__module__ == np.__name__:
            if type(x) == np.ndarray:
                return recurse(list(x), helper)
            return np.asscalar(x)
        return x
    return recurse(x, helper)

def smooth(y, box_pts):
    box = np.ones(box_pts) / box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

def normalize(x, eps=1e-8):
    return (x - x.mean()) / x.std()

##### torch functions

try:
    import torch
    import torch.nn as nn
    import torch.nn.functional as F
    import torch.optim as optim
    from torch.utils.data import Dataset, DataLoader

    def to_torch(x, device='cuda' if torch.cuda.is_available() else 'cpu', **kwargs):
        def helper(x):
            if x is None:
                return None
            elif type(x) == torch.Tensor:
                return x.to(device=device, **kwargs)
            elif type(x) in [str, bool, int, float]:
                return x
            return torch.from_numpy(x).to(device=device, **kwargs)
        return recurse(x, helper)

    def from_torch(t, force_scalar=False):
        def helper(t):
            if type(t) != torch.Tensor:
                return t
            x = t.detach().cpu().numpy()
            if force_scalar and (x.size == 1 or np.isscalar(x)):
                return np.asscalar(x)
            return x
        return recurse(t, helper)

    def count_params(network, requires_grad=False):
        return sum(p.numel() for p in network.parameters() if not requires_grad or p.requires_grad)

    def report_memory(device=None, max=False):
        if device:
            device = torch.device(device)
            if max:
                alloc = torch.cuda.max_memory_allocated(device=device)
            else:
                alloc = torch.cuda.memory_allocated(device=device)
            alloc /=  1024 ** 2
            print('%.3f MBs' % alloc)
            return alloc

        numels = Counter()
        for obj in gc.get_objects():
            if torch.is_tensor(obj):
                print(type(obj), obj.size())
                numels[obj.device] += obj.numel()
        print()
        for device, numel in sorted(numels.items()):
            print('%s: %s elements, %.3f MBs' % (str(device), numel, numel * 4 / 1024 ** 2))

    def clear_gpu_memory():
        gc.collect()
        torch.cuda.empty_cache()
        for obj in gc.get_objects():
            if torch.is_tensor(obj):
                obj.cpu()
        gc.collect()
        torch.cuda.empty_cache()

    def gelu(x):
        return 0.5 * x * (1 + torch.tanh(np.sqrt(2 / np.pi) * (x + 0.044715 * torch.pow(x, 3))))

    class GeLU(nn.Module):
        def forward(self, input):
            return gelu(input)

    class Flatten(nn.Module):
        def forward(self, input):
            return input.view(input.size(0), -1)

    class Reshape(nn.Module):
        def __init__(self, *shape, split=None, merge=None):
            super(Reshape, self).__init__()
            self.shape = shape
            self.split = split
            self.merge = merge

        def forward(self, input):
            if self.split is None and self.merge is None:
                return input.reshape(*self.shape)
            in_shape = input.shape

    class Transpose(nn.Module):
        def __init__(self, dim0, dim1):
            super(Transpose, self).__init__()
            self.dim0 = dim0
            self.dim1 = dim1

        def forward(self, input):
            return input.transpose(self.dim0, self.dim1)

    class Permute(nn.Module):
        def __init__(self, *dims):
            super(Permute, self).__init__()
            self.dims = dims

        def forward(self, input):
            return input.permute(*self.dims)

    class Attention(nn.Module):
        def __init__(self, n_io, n_k, n_v=None, n_head=1, n_ctx=None, layer_norm=False):
            super(Attention, self).__init__()
            self.n_k = n_k
            self.n_v = n_v = n_v or n_k
            self.n_head = n_head
            self.n_ctx = n_ctx
            self.layer_norm = layer_norm
            if layer_norm:
                self.ln = nn.LayerNorm(n_io)
            self.fc_qkv = nn.Linear(n_io, n_head * (n_k * 2 + n_v))
            self.fc_out = nn.Linear(n_head * n_v, n_io)

        def merge_past(self, kv, past_kv, qk_ignore):
            if past_kv is None:
                return kv, qk_ignore
            past_ignore = torch.zeros((qk_ignore.size(0), past_kv.size(2)))
            return (
                torch.cat((past_kv, kv), dim=2),
                torch.cat((past_ignore, qk_ignore), dim=1)
            )

        def attend(self, qk, qk_ignore):
            qk.data.masked_fill_(qk_ignore, -np.inf)
            attn_weights = qk.softmax(dim=-1)
            return attn_weights

        def forward(self, input, past_kv=None):
            n_b, n_ctx, n_io = input.shape
            n_head = self.n_head
            n_k, n_v = self.n_k, self.n_v
            if self.layer_norm:
                input = self.ln(input)

            q_kv = self.fc_qkv(input).reshape(n_b, n_ctx, n_head, -1).split([n_k, n_k + n_v], dim=-1)
            q, kv = map(lambda x: x.transpose(1, 2), q_kv) # shape (n_b, n_head, n_ctx, n_kv)

            qk_ignore = torch.triu(torch.ones((n_ctx, n_ctx)), diagonal=1)
            kv, qk_ignore = self.merge_past(kv, past_kv, qk_ignore)

            k, v = kv.split([n_k, n_v], dim=-1)
            qk = torch.einsum('bhck,bhdk->bhcd', q, k) / np.sqrt(n_k)

            qk_ignore = qk_ignore.byte().reshape(1, 1, *qk_ignore.shape).to(qk.device)
            attn_weights = self.attend(qk, qk_ignore)
            qkv_out = torch.einsum('bhcd,bhdk->bchk', attn_weights, v)

            out = self.fc_out(qkv_out.reshape(n_b, n_ctx, n_head * n_v))
            return out, kv

        def forward_all(self, input):
            n_b, n_seq, n_io = input.shape
            n_head = self.n_head
            n_ctx = min(self.n_ctx or n_seq, n_seq)
            n_k, n_v = self.n_k, self.n_v
            if self.layer_norm:
                input = self.ln(input)

            q_kv = self.fc_qkv(input).reshape(n_b, n_seq, n_head, -1).split([n_k, n_k + n_v], dim=-1)
            qkv_outs = []
            past_kv = None
            for q, kv in zip(*map(lambda x: x.transpose(1, 2).split(n_ctx, dim=2), q_kv)):
                # each of q, k, v is shape (n_b, n_head, n_ctx, n_kv)
                qk_ignore = torch.triu(torch.ones((n_ctx, n_ctx)), diagonal=1)
                kv, qk_ignore = self.merge_past(kv, past_kv, qk_ignore)

                k, v = kv.split([n_k, n_v], dim=-1)
                qk = torch.einsum('bhsk,bhck->bhsc', q, k) / np.sqrt(n_k)
                qk_ignore = qk_ignore.byte().reshape(1, 1, *qk_ignore.shape).to(qk.device)
                attn_weights = self.attend(qk, qk_ignore)

                qkv_outs.append(torch.einsum('bhsc,bhcv->bshv', attn_weights, v))
                past_kv = kv
            qkv_out = torch.cat(qkv_outs, dim=1)
            out = self.fc_out(qkv_out.reshape(n_b, n_seq, n_head * n_v))
            return out, past_kv

    class CausalConv1d(nn.Module):
        def __init__(self, in_depth, out_depth, kernel_size, dilation=1, stride=1, groups=1):
            super(CausalConv1d, self).__init__()
            self.padding = (kernel_size - 1) * dilation
            self.conv = nn.Conv1d(in_depth, out_depth, kernel_size, stride=stride, dilation=dilation, groups=groups)

        def forward(self, x, pad=True):
            if pad:
                x = F.pad(x, (self.padding, 0))
            return self.conv(x)

    class CausalMaxPool1d(nn.Module):
        def __init__(self, kernel_size, dilation=1, stride=1):
            super(CausalMaxPool1d, self).__init__()
            self.padding = (kernel_size - 1) * dilation
            self.pool = nn.MaxPool1d(kernel_size, stride=stride, dilation=dilation)

        def forward(self, x, pad=True):
            if pad:
                x = F.pad(x, (self.padding, 0))
            return self.pool(x)


except ImportError:
    pass
