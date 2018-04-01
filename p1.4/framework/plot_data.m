function [a] = plot_data(h, labely, key1, varargin)
    keys = get_plot_keys;
    m = containers.Map(keys, [1:length(keys)]);
    if ~isempty(varargin)
        key2 = varargin{1};
        a=plot(h(:,m('t')), h(:,m(key1)), 'b', h(:,m('t')),h(:,m(key2)),'r');
    else
        a=plot(h(:,m('t')), h(:,m(key1)), 'b');
    end
    ylabel(labely);
end
