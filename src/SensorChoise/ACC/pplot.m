function handle = pplot(Data,iter)

handle = plot(real(Data),imag(Data),iter)
xlabel('Real')
ylabel('Imag')
grid on
end