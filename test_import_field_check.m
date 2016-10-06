clear all

a(1).b = 1;
a(3).b = [];

b(1).b = 3;
b(3).b = 2;


b(2).c = 3;


a_names = fieldnames(a);
b_names = fieldnames(b);

for ii = 1:length(b_names)
    if ~any(strcmp(b_names{ii},a_names))
        a(kk).(b_names{ii}) = [];
    end
    for kk = 1:length(b)
        if isempty(a(kk).(b_names{ii}))
            a(kk).(b_names{ii}) = b(kk).(b_names{ii});
        end
    end
end

        