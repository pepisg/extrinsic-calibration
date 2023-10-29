packages=$(colcon list -n)
_pkg_build_autocomplete()
{
    local cur=${COMP_WORDS[$COMP_CWORD]}
    COMPREPLY=( $(compgen -W "$packages" -- $cur) )
}

alias pkg-build-up-to="source ~/scripts/pkg_build.sh --symlink-install --packages-up-to"
alias pkg-build-select="source ~/scripts/pkg_build.sh --symlink-install --packages-select"

complete -F _pkg_build_autocomplete pkg-build-up-to
complete -F _pkg_build_autocomplete pkg-build-select