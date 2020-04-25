import { ConfigInterface } from '../config/config.interface';
import { LogService } from './log.service';
import { EventMpptChg, MpptchgService } from './mpptchg.service';
import { AprsService } from './aprs.service';
import { SensorsService } from './sensors.service';
import { of, Subscription, timer } from 'rxjs';
import { catchError, skip, switchMap, tap } from 'rxjs/operators';
import { VoiceService } from './voice.service';
import { AudioDecoder, MultimonModeEnum } from 'nodejs-arecord-multimon';
import { WebcamService } from './webcam.service';
import { SstvService } from './sstv.service';
import { RadioService } from './radio.service';
import { DatabaseService } from './database.service';
import { ApiService } from './api.service';
import { exec } from 'child_process';
import { GpioService } from './gpio.service';
import { CommunicationMpptchdService } from './communication-mpptchd.service';

export class ProcessService {

    public static debug: boolean;

    private audioDecoder = new AudioDecoder();
    private mpptchgSubscription: Subscription;
    private api: ApiService;

    private stopDirectly: boolean;

    public run(config: ConfigInterface): void {
        ProcessService.debug = !!config.debug;

        LogService.LOG_PATH = config.logsPath;
        GpioService.USE_FAKE = !!config.fakeGpio;

        if (config.mpptChd) {
            CommunicationMpptchdService.USE_FAKE = !!config.mpptChd.fake;
            CommunicationMpptchdService.DEBUG = !!config.mpptChd.debugI2C;
        }

        if (config.webcam) {
            WebcamService.USE_FAKE = !!config.webcam.fake;
        }

        LogService.log('program', 'Starting');

        try {
            DatabaseService.openDatabase(config.logsPath).pipe(
                switchMap(_ => RadioService.pttOff(true))
            ).subscribe(_ => {
                if (config.mpptChd && config.mpptChd.enable) {
                    this.runMpptChd(config);
                }

                if (config.aprs && config.aprs.enable) {
                    this.runAprs(config);
                }

                if (config.sensors && config.sensors.enable) {
                    this.runSensors(config);
                }

                if (config.webcam && config.webcam.enable) {
                    this.runWebcam(config);
                }

                if (config.api && config.api.enable) {
                    this.runApi(config);
                }

                if (config.sftp && config.sftp.enable) {
                    this.runSftp(config);
                }

                process.on('exit', event => this.exitHandler(config, event));
                process.on('SIGINT', event => this.exitHandler(config, event));
                process.on('SIGTERM', event => this.exitHandler(config, event));
                process.on('SIGQUIT', event => this.exitHandler(config, event));

                LogService.log('program', 'Started');
            });
        } catch (e) {
            this.exitHandler(config, 0);
        }
    }

    private exitHandler(config: ConfigInterface, event) {
        let stop = of(null);

        if (!this.stopDirectly) {
            LogService.log('program', 'Stopping', event);

            if (this.mpptchgSubscription) {
                this.mpptchgSubscription.unsubscribe();

                if (event !== EventMpptChg.ALERT) {
                    stop = stop.pipe(
                        switchMap(_ =>
                            MpptchgService.stopWatchdog().pipe(
                                tap(_ => LogService.log('mpptChd', 'Watchdog disabled if enabled')),
                                catchError(e => {
                                    LogService.log('mpptChd', 'Watchdog impossible to disabled (if enabled) !');
                                    return of(null);
                                })
                            )
                        )
                    );
                }
            }

            if (config.aprs && config.aprs.enable) {
                stop = stop.pipe(
                    switchMap(_ => AprsService.sendAprsBeacon(config.aprs, true)),
                    switchMap(_ => AprsService.sendAprsTelemetry(config.aprs, config.sensors))
                );
            }

            stop = stop.pipe(
                switchMap(_ => RadioService.pttOff(true)),
                catchError(err => {
                    LogService.log('program', 'Stopped KO', err);
                    return RadioService.pttOff(true).pipe(catchError(_ => of(null)));
                })
            );
        }

        stop.subscribe(_ => {
            if (this.stopDirectly) {
                LogService.log('program', 'Stopped', event);
            }
            this.stopDirectly = true;
            if (!ProcessService.debug) {
                exec('halt');
            }
            process.exit(0);
        });
    }

    private runMpptChd(config: ConfigInterface): void {
        this.mpptchgSubscription = MpptchgService.battery(config).subscribe();
        MpptchgService.events.on(EventMpptChg.ALERT, _ => this.exitHandler(config, EventMpptChg.ALERT));
    }

    private runAprs(config: ConfigInterface): void {
        LogService.log('aprs', 'Started');
        timer(60000, 1000 * (config.aprs.interval ? config.aprs.interval : 900)).subscribe(_ =>
            AprsService.sendAprsBeacon(config.aprs, true).pipe(
                switchMap(_ => AprsService.sendAprsTelemetry(config.aprs, config.sensors, true))
            ).subscribe(_ => !!config.aprs.waitDtmfInterval && !AprsService.alreadyInUse ? this.runDtmfDecoder(config) : null)
        );
    }

    private runDtmfDecoder(config: ConfigInterface): void {
        LogService.log('dtmf', 'Start listening');
        let dtmfCode = '';
        let shouldStop = true;

        const decoderSubscription = this.audioDecoder.decode(config.audioDevice, [MultimonModeEnum.DTMF, MultimonModeEnum.TONE], [], ['-T 1750']).pipe(
            skip(1)
        ).subscribe(result => {
            if (shouldStop) {
                LogService.log('dtmf', 'Data decoded', result);

                if (result.type === MultimonModeEnum.TONE) {
                    dtmfCode = '';
                    if (config.voice && config.voice.enable) {
                        shouldStop = false;
                        VoiceService.sendVoice(config.voice.sentence, true).subscribe(_ => shouldStop = true);
                    }
                } else if (result.type === MultimonModeEnum.DTMF) {
                    if (result.data === '#') {
                        LogService.log('dtmf', 'Reset code', dtmfCode);
                        dtmfCode = '';
                    } else if (result.data === '*') {
                        shouldStop = false;
                        if (dtmfCode === config.sstv.dtmfCode) {
                            if (config.sstv && config.sstv.enable) {
                                SstvService.sendImage(config.sstv, true).subscribe(_ => shouldStop = true);
                            } else {
                                LogService.log('dtmf', 'Function disabled', dtmfCode);
                                VoiceService.sendVoice('Fonction demandée désactivée. ' + config.voice.sentence, true).subscribe(_ => shouldStop = true);
                            }
                        } else {
                            LogService.log('dtmf', 'Code not recognized', dtmfCode);
                            VoiceService.sendVoice('Erreur code, ' + dtmfCode + '. ' + config.voice.sentence, true).subscribe(_ => shouldStop = true);
                        }
                        dtmfCode = '';
                    } else if (!dtmfCode.endsWith(result.data)) {
                        dtmfCode += result.data;
                    }
                }
            }
        });

        const interval = 1000 * (config.aprs.waitDtmfInterval ? config.aprs.waitDtmfInterval : 60);
        const timerSubscription = timer(interval, interval).subscribe(_ => {
            if (shouldStop) {
                LogService.log('dtmf', 'Timeout');
                this.audioDecoder.stop().subscribe();
                RadioService.switchOff().subscribe();
                decoderSubscription.unsubscribe();
                timerSubscription.unsubscribe();
            } else {
                LogService.log('dtmf', 'Should timeout, restart', interval);
            }
        });
    }

    private runSensors(config: ConfigInterface): void {
        LogService.log('sensors', 'Started');
        timer(60000, 1000 * (config.sensors.interval ? config.sensors.interval : 60)).subscribe(_ =>
            SensorsService.getAllAndSave(config.sensors).subscribe()
        );
    }

    private runWebcam(config: ConfigInterface): void {
        LogService.log('webcam', 'Started');
        timer(30000, 1000 * (config.webcam.interval ? config.webcam.interval : 30)).subscribe(_ =>
            WebcamService.captureAndSend(config.webcam, config.sftp).subscribe()
        );
    }

    private runApi(config: ConfigInterface): void {
        this.api = new ApiService(config);
    }

    private runSftp(config: ConfigInterface): void {
        LogService.log('sftp', 'Started');
        timer(60000, 1000 * (config.sftp.interval ? config.sftp.interval : 60)).subscribe(_ => {
            LogService.send(config.sftp, config.logsPath).subscribe();
        });
    }
}
