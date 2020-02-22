"------------------------ Function definitions ------------------------
" Creates the specified directory if doesn't exist.
" @param dirpath The directory path
function s:mkdir(dirpath)
	if !isdirectory(a:dirpath)
		call mkdir(expand(a:dirpath), 'p')
	endif
endfunction

" Clones git repository to specified path.
" Usage: call s:githubclone('takumi/repo.git', '~/.vim/cache')
" @param repo    Github repository name: takumi/repo.git (means https://github.com/takumi/repo.git)
" @param dirpath Path like: ~/.vim/cache (repository will be ~/.vim/cache/repo)
function s:githubclone(repo, dirpath)
	if !isdirectory(expand(a:dirpath . '/' . substitute(split(a:repo, '/')[-1], '.git$', '', 'g')))
		execute 'silent !cd ' . a:dirpath . ' ; git clone https://github.com/' . a:repo
	endif
	return expand(a:dirpath . '/' . substitute(split(a:repo, '/')[-1], '.git$', '', 'g'))
endfunction

" Sets value to option variable
" @param option Option name like: backupdir
" @param valu   Value to set to the specified option
function s:setoption(option, value)
	execute 'set ' . a:option . '=' . a:value
endfunction


"------------------------ Basic directories ------------------------
let s:cache_dir  = expand('~/.vim/cache')
let s:tmp_dir    = expand('~/.vim/tmp')
let s:bundle_dir = expand('~/.vim/bundles')
" check if directories exist and create it
call s:mkdir(s:tmp_dir)
call s:mkdir(s:cache_dir)
call s:mkdir(s:bundle_dir)
" set auto output files directory
call s:setoption('directory', s:tmp_dir)
call s:setoption('backupdir', s:tmp_dir)
call s:setoption('viminfo+', 'n~/.vim/viminfo')


"------------------------ Basic settings ------------------------
let s:colorscheme_plugin = 'jacoborus/tender'
set number                        " display line number
set whichwrap=b,s,h,l,<,>,[,],~   " automatically wrap left and right
set tabstop=4                     " tab width
set shiftwidth=4
set backspace=start,eol,indent    " backspace and delete problems
set cursorline                    " hightlite cursor line
set ambiwidth=double              " for double-byte character
set t_Co=256
" bash like completion method
set wildmenu
set wildignorecase
set wildmode=longest,list


"------------------------ Plugin settings ------------------------
" Set up dein plugin manager
let s:dein_path = s:bundle_dir . '/dein.vim'
if &compatible
	set nocompatible
endif
if !isdirectory(s:dein_path)
	" Download if it isn't exist
	execute 'silent !echo "-------- Downloading Dein plugin manager --------"'
	call s:githubclone('Shougo/dein.vim', s:bundle_dir)
endif
call s:setoption('runtimepath+', s:dein_path)
if dein#load_state(s:bundle_dir)
	call dein#begin(s:bundle_dir)
	call dein#add(s:dein_path)

	" status line
	call dein#add('itchyny/lightline.vim')
	" Colorscheme
	call dein#add(s:colorscheme_plugin . '.vim')
	
	call dein#add('tyru/caw.vim')

	call dein#end()
	call dein#save_state()
endif
filetype plugin indent on
if dein#check_install()
	call dein#install()
endif
syntax on
execute 'colorscheme ' . split(s:colorscheme_plugin, '/')[1]


"------------------------ lightline and tabline setup ------------------------
" Returns current buffer name.
" This function uses fnamebody()
" @param mods 2nd argument for fnamemodify()
" @return     Current buffer name
function! g:LL_getCurrentBufferName(mods)
	let n = tabpagenr()
	let bufnrs = tabpagebuflist(n)
	let curbufnr = bufnrs[tabpagewinnr(n) - 1]
	let fname = bufname(curbufnr)
	let path = fnamemodify(fname, a:mods)
	return path
endfunction
" activate and basic settings
set noshowmode
set laststatus=2
set showtabline=2
set guioptions-=e
" customize lightline
let g:lightline = {'colorscheme': 'wombat'}
let g:lightline.component = {
	\ 'bufpath': '%{g:LL_getCurrentBufferName(:~)}',
	\ 'bufdir': '%{g:LL_getCurrentBufferName(":~:h")}'
\ }
let g:lightline.tabline = {
	\ 'right': [['bufdir']]
\ }
let g:lightline.tab = {
	\ 'active': ['filename', 'modified'],
	\ 'inactive': ['filename', 'modified']
\ }


highlight Visual ctermbg=LightGray guibg=LightGray

" C-* key settings
if has('unix') || has('mac')
	" enable C-s, C-q keybind
	execute 'silent !stty -ixon'
	execute 'silent !stty quit undef'
endif
if has('win32') || has('win64')
endif

" toggle comment
nmap <C-\> <Plug>(caw:hatpos:toggle)
imap <C-\> <ESC><Plug>(caw:hatpos:toggle)a
vmap <C-\> <Plug>(caw:hatpos:toggle)

" save
nnoremap <C-s> :w<CR>
inoremap <C-s> <ESC>:w<CR>a
cnoremap <C-s> <C-e><C-u>w<CR>
" exit
nnoremap <C-q> :q<CR>
inoremap <C-q> <ESC>:q<CR>
cnoremap <C-q> <C-e><C-u>q<CR>

" new tab
nnoremap <C-n> :tabedit 
inoremap <C-n> <ESC>:tabedit 
cnoremap <C-n> <C-e><C-u>tabedit 
" move tab (<-)
nnoremap <C-h> :tabprevious<CR>
inoremap <C-h> <ESC>:tabprevious<CR>
cnoremap <C-h> <C-e><C-u>tabprevious<CR>
" move tab (->)
nnoremap <C-l> :tabnext<CR>
inoremap <C-l> <ESC>:tabnext<CR>
cnoremap <C-l> <C-e><C-u>tabnext<CR>

" half page move (v)
nnoremap <C-j> <C-d>zz
inoremap <C-j> <ESC><C-d>zza
cnoremap <C-j> <C-e><C-u><BS><C-d>zz
" half page move (^)
nnoremap <C-k> <C-u>zz
inoremap <C-k> <ESC><C-u>zza
cnoremap <C-k> <C-e><C-u><BS><C-u>zz
" move to head
nnoremap <C-a> ^
inoremap <C-a> <ESC>^i
" move to tail
nnoremap <C-e> $
inoremap <C-e> <ESC>$a
" undo
nnoremap <C-z> u
inoremap <C-z> <ESC>ua
cnoremap <C-z> <C-e><C-u>undo<CR>
" redo
nnoremap <C-y> <C-r>
inoremap <C-y> <ESC><C-r>a
cnoremap <C-y> <C-e><C-u>redo<CR>

" cut, copy, paste
vnoremap <C-x> d
vnoremap <C-c> y
nnoremap <C-v> p
inoremap <C-v> <ESC>pa

" regex search
nnoremap <C-f> /\v
inoremap <C-f> <ESC>/\v

" terminal
nnoremap <C-t> :term bash<CR>
inoremap <C-t> <ESC>:term bash<CR>


